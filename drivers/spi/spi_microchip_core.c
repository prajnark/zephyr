/*
 * Copyright (c) 2025 Microchip Technology
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * This is a straight port of the Linux driver logic you shared, adapted to
 * Zephyr's SPI driver model and devicetree API. It currently implements a
 * blocking (polling) transfer path similar to your Linux version's
 * write_fifo/read_fifo loop. IRQs are wired, but only used for error clears.
 *
 * TODOs you may want to expand later:
 *  - Multi-CS selection (currently assumes CS#0 when using internal SSEL)
 *  - Optional use of cs-gpios instead of internal SSEL, if present in DT
 *  - Async transfers (CONFIG_SPI_ASYNC) using spi_context + ISR signalling
 *  - More exhaustive mode handling beyond Motorola mode check
 */

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

LOG_MODULE_REGISTER(mchp_corespi, CONFIG_SPI_LOG_LEVEL);

/* Match DT nodes with compatible = "microchip,corespi-rtl-v5" */
#define DT_DRV_COMPAT microchip_corespi_rtl_v5

/* --- HW defs copied from your Linux driver --- */
#define MCHP_CORESPI_MAX_CS                 (8)
#define MCHP_CORESPI_DEFAULT_FIFO_DEPTH     (4)
#define MCHP_CORESPI_DEFAULT_MOTOROLA_MODE  (3)

#define MCHP_CORESPI_CONTROL_ENABLE         BIT(0)
#define MCHP_CORESPI_CONTROL_MASTER         BIT(1)
#define MCHP_CORESPI_CONTROL_TX_DATA_INT    BIT(3)
#define MCHP_CORESPI_CONTROL_RX_OVER_INT    BIT(4)
#define MCHP_CORESPI_CONTROL_TX_UNDER_INT   BIT(5)
#define MCHP_CORESPI_CONTROL_FRAMEURUN      BIT(6)
#define MCHP_CORESPI_CONTROL_OENOFF         BIT(7)

#define MCHP_CORESPI_STATUS_ACTIVE          BIT(7)
#define MCHP_CORESPI_STATUS_SSEL            BIT(6)
#define MCHP_CORESPI_STATUS_TXFIFO_UNDERFLOW BIT(5)
#define MCHP_CORESPI_STATUS_RXFIFO_FULL     BIT(4)
#define MCHP_CORESPI_STATUS_TXFIFO_FULL     BIT(3)
#define MCHP_CORESPI_STATUS_RXFIFO_EMPTY    BIT(2)
#define MCHP_CORESPI_STATUS_DONE            BIT(1)
#define MCHP_CORESPI_STATUS_FIRSTFRAME      BIT(0)

#define MCHP_CORESPI_INT_TXDONE             BIT(0)
#define MCHP_CORESPI_INT_RX_CHANNEL_OVERFLOW BIT(2)
#define MCHP_CORESPI_INT_TX_CHANNEL_UNDERRUN BIT(3)
#define MCHP_CORESPI_INT_CMDINT             BIT(4)
#define MCHP_CORESPI_INT_SSEND              BIT(5)
#define MCHP_CORESPI_INT_DATA_RX            BIT(6)
#define MCHP_CORESPI_INT_TXRFM              BIT(7)

#define MCHP_CORESPI_CONTROL2_INTEN_TXRFMT  BIT(7)
#define MCHP_CORESPI_CONTROL2_INTEN_DATA_RX BIT(6)
#define MCHP_CORESPI_CONTROL2_INTEN_SSEND   BIT(5)
#define MCHP_CORESPI_CONTROL2_INTEN_CMD     BIT(4)

#define INT_ENABLE_MASK (MCHP_CORESPI_CONTROL_TX_DATA_INT | \
                         MCHP_CORESPI_CONTROL_RX_OVER_INT | \
                         MCHP_CORESPI_CONTROL_TX_UNDER_INT)

#define MCHP_CORESPI_REG_CONTROL            (0x00)
#define MCHP_CORESPI_REG_INTCLEAR           (0x04)
#define MCHP_CORESPI_REG_RXDATA             (0x08)
#define MCHP_CORESPI_REG_TXDATA             (0x0c)
#define MCHP_CORESPI_REG_INTMASK            (0x10)
#define MCHP_CORESPI_REG_INTRAW             (0x14)
#define MCHP_CORESPI_REG_CONTROL2           (0x18)
#define MCHP_CORESPI_REG_COMMAND            (0x1c)
#define MCHP_CORESPI_REG_STAT               (0x20)
#define MCHP_CORESPI_REG_SSEL               (0x24)
#define MCHP_CORESPI_REG_TXDATA_LAST        (0x28)
#define MCHP_CORESPI_REG_CLK_DIV            (0x2c)

/* --- Zephyr driver structs --- */
struct mchp_corespi_config {
	mm_reg_t base;
	void (*irq_config_func)(const struct device *dev);
	uint32_t clock_freq;      /* peripheral clock in Hz (from DT) */
	uint8_t fifo_depth;       /* optional, default 4 */
	uint8_t motorola_mode;    /* 0..3, default 3 */
	bool use_internal_ssel;   /* if true, driver toggles SSEL register */
};

struct mchp_corespi_data {
	struct spi_context ctx;
	/* cached transfer params */
	uint8_t n_bytes; /* bytes per word: 1,2,4 */
};

/* --- MMIO helpers --- */
static inline uint8_t spiread8(const struct mchp_corespi_config *cfg, mm_reg_t off)
{
	return sys_read8(cfg->base + off);
}

static inline void spiwrite8(const struct mchp_corespi_config *cfg, uint8_t v, mm_reg_t off)
{
	sys_write8(v, cfg->base + off);
}

/* --- Core helpers --- */
static inline void corespi_disable(const struct mchp_corespi_config *cfg)
{
	uint8_t c = spiread8(cfg, MCHP_CORESPI_REG_CONTROL);
	c &= ~MCHP_CORESPI_CONTROL_ENABLE;
	spiwrite8(cfg, c, MCHP_CORESPI_REG_CONTROL);
}

static inline void corespi_enable(const struct mchp_corespi_config *cfg)
{
	uint8_t c = spiread8(cfg, MCHP_CORESPI_REG_CONTROL);
	c |= MCHP_CORESPI_CONTROL_ENABLE;
	spiwrite8(cfg, c, MCHP_CORESPI_REG_CONTROL);
}

static inline void corespi_enable_ints(const struct mchp_corespi_config *cfg)
{
	uint8_t c = spiread8(cfg, MCHP_CORESPI_REG_CONTROL);
	c |= INT_ENABLE_MASK;
	spiwrite8(cfg, c, MCHP_CORESPI_REG_CONTROL);
}

static inline void corespi_disable_ints(const struct mchp_corespi_config *cfg)
{
	uint8_t c = spiread8(cfg, MCHP_CORESPI_REG_CONTROL);
	c &= ~INT_ENABLE_MASK;
	spiwrite8(cfg, c, MCHP_CORESPI_REG_CONTROL);
}

static inline void corespi_assert_ssel(const struct mchp_corespi_config *cfg, bool assert)
{
	if (!cfg->use_internal_ssel) {
		return; /* assume cs-gpios handled by spi_context */
	}
	uint8_t reg = spiread8(cfg, MCHP_CORESPI_REG_SSEL);
	/* Use CS#0 for now. Set bit to 1 when active, same as your Linux logic. */
	reg &= ~BIT(0);
	reg |= (assert ? 1U : 0U) << 0;
	spiwrite8(cfg, reg, MCHP_CORESPI_REG_SSEL);
}

static int corespi_set_clk_div(const struct mchp_corespi_config *cfg, uint32_t target_hz)
{
	uint32_t pclk = cfg->clock_freq;
	if (!pclk || !target_hz) {
		return -EINVAL;
	}
	/* SPICLK = PCLK / (2 * (CLK_DIV + 1)) */
	uint32_t clk_div = DIV_ROUND_UP(pclk, 2U * target_hz) - 1U;
	if (clk_div > 0xFFU) {
		return -EINVAL;
	}
	/* Recompute actual rate to sanity-check */
	uint32_t spi_hz = pclk / (2U * (clk_div + 1U));
	if (spi_hz > target_hz) {
		/* too fast, reject */
		return -EINVAL;
	}
	spiwrite8(cfg, (uint8_t)clk_div, MCHP_CORESPI_REG_CLK_DIV);
	return 0;
}

static int corespi_check_mode(const struct mchp_corespi_config *cfg, uint16_t op)
{
	/* Only Motorola mode N (CPOL/CPHA pair) supported; must match DT default */
	bool cpol = (op & SPI_MODE_CPOL) != 0;
	bool cpha = (op & SPI_MODE_CPHA) != 0;
	uint8_t mode = (cpol ? 2 : 0) | (cpha ? 1 : 0); /* 0..3 */
	if (mode != cfg->motorola_mode) {
		LOG_ERR("Incompatible CPOL/CPHA: controller fixed to mode %u", cfg->motorola_mode);
		return -ENOTSUP;
	}
	if (op & SPI_CS_ACTIVE_HIGH) {
		LOG_ERR("Active-high CS not supported by controller Motorola mode");
		return -ENOTSUP;
	}
	if (op & (SPI_MODE_LOOP | SPI_OP_MODE_SLAVE)) {
		return -ENOTSUP;
	}
	return 0;
}

static inline uint8_t word_bytes_from_op(uint32_t op)
{
	uint8_t w = SPI_WORD_SIZE_GET(op);
	if (w == 0) {
		w = 8; /* default */
	}
	return MAX( (uint8_t)1, (uint8_t)DIV_ROUND_UP(w, 8) );
}

/* Simple polling write/read, analogous to your Linux transfer_one loop */
static inline void corespi_write_fifo(struct spi_context *ctx,
                                     const struct mchp_corespi_config *cfg,
                                     uint8_t n_bytes, uint32_t fifo_max)
{
	uint32_t i = 0;
	while (i < fifo_max && !(spiread8(cfg, MCHP_CORESPI_REG_STAT) & MCHP_CORESPI_STATUS_TXFIFO_FULL)) {
		uint32_t word = 0xAA; /* default filler */
		if (spi_context_tx_buf_on(ctx)) {
			if (n_bytes == 4) {
				word = UNALIGNED_GET((uint32_t *)ctx->tx_buf);
			} else if (n_bytes == 2) {
				word = UNALIGNED_GET((uint16_t *)ctx->tx_buf);
			} else {
				word = *ctx->tx_buf;
			}
		}
		/* If last frame, use TXDATA_LAST so core deasserts SSEL automatically */
		bool last_frame = (ctx->tx_len == n_bytes);
		spiwrite8(cfg, (uint8_t)word,
			  last_frame ? MCHP_CORESPI_REG_TXDATA_LAST : MCHP_CORESPI_REG_TXDATA);
		if (spi_context_tx_buf_on(ctx)) {
			spi_context_update_tx(ctx, 1, n_bytes);
		}
		i++;
	}
}

static inline void corespi_read_fifo(struct spi_context *ctx,
                                    const struct mchp_corespi_config *cfg,
                                    uint8_t n_bytes, uint32_t fifo_max)
{
	for (uint32_t i = 0; i < fifo_max; i++) {
		while (spiread8(cfg, MCHP_CORESPI_REG_STAT) & MCHP_CORESPI_STATUS_RXFIFO_EMPTY) {
			; /* spin */
		}
		uint32_t data = spiread8(cfg, MCHP_CORESPI_REG_RXDATA);
		if (spi_context_rx_buf_on(ctx)) {
			if (n_bytes == 4) {
				UNALIGNED_PUT((uint32_t)data, (uint32_t *)ctx->rx_buf);
			} else if (n_bytes == 2) {
				UNALIGNED_PUT((uint16_t)data, (uint16_t *)ctx->rx_buf);
			} else {
				*ctx->rx_buf = (uint8_t)data;
			}
			spi_context_update_rx(ctx, 1, n_bytes);
		}
	}
}

static int corespi_transceive(const struct device *dev,
                              const struct spi_config *spi_cfg,
                              const struct spi_buf_set *tx_bufs,
                              const struct spi_buf_set *rx_bufs,
                              bool async,
                              spi_callback_t cb,
                              void *userdata)
{
	ARG_UNUSED(async);
	ARG_UNUSED(cb);
	ARG_UNUSED(userdata);

	const struct mchp_corespi_config *cfg = dev->config;
	struct mchp_corespi_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	int ret;

	spi_context_lock(ctx, false, NULL, NULL, spi_cfg);

	ret = corespi_check_mode(cfg, spi_cfg->operation);
	if (ret) {
		goto out;
	}

	/* Program clock */
	ret = corespi_set_clk_div(cfg, spi_cfg->frequency);
	if (ret) {
		LOG_ERR("Failed to set clock for %u Hz", spi_cfg->frequency);
		goto out;
	}

	/* Determine word size */
	data->n_bytes = word_bytes_from_op(spi_cfg->operation);

	/* Setup context with user buffers */
	spi_context_buffers_setup(ctx, tx_bufs, rx_bufs, data->n_bytes);

	/* Assert CS (GPIO) if provided */
	spi_context_cs_control(ctx, true);
	/* Or internal SSEL if requested */
	corespi_assert_ssel(cfg, true);

	while (ctx->tx_len) {
		uint32_t bytes_left = ctx->tx_len;
		uint32_t fifo_max = DIV_ROUND_UP(MIN(bytes_left, cfg->fifo_depth), data->n_bytes);
		corespi_write_fifo(ctx, cfg, data->n_bytes, fifo_max);
		corespi_read_fifo(ctx, cfg, data->n_bytes, fifo_max);
	}

	/* Deassert CS */
	corespi_assert_ssel(cfg, false);
	spi_context_cs_control(ctx, false);

	ret = 0;

out:
	spi_context_release(ctx, ret);
	return ret;
}

static int corespi_transceive_blocking(const struct device *dev,
                                       const struct spi_config *spi_cfg,
                                       const struct spi_buf_set *tx_bufs,
                                       const struct spi_buf_set *rx_bufs)
{
	return corespi_transceive(dev, spi_cfg, tx_bufs, rx_bufs, false, NULL, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int corespi_transceive_async(const struct device *dev,
                                    const struct spi_config *spi_cfg,
                                    const struct spi_buf_set *tx_bufs,
                                    const struct spi_buf_set *rx_bufs,
                                    spi_callback_t cb,
                                    void *userdata)
{
	return corespi_transceive(dev, spi_cfg, tx_bufs, rx_bufs, true, cb, userdata);
}
#endif /* CONFIG_SPI_ASYNC */

static int corespi_release(const struct device *dev, const struct spi_config *cfg)
{
	ARG_UNUSED(cfg);
	struct mchp_corespi_data *data = dev->data;
	spi_context_unlock_unconditionally(&data->ctx);
	return 0;
}

static void corespi_isr(const struct device *dev)
{
	const struct mchp_corespi_config *cfg = dev->config;
	/* Clear/ack simple status as in Linux handler; don’t complete here */
	uint8_t intfield = spiread8(cfg, MCHP_CORESPI_REG_INTMASK) & 0xff;
	if (!intfield) {
		return; /* spurious */
	}
	if (intfield & MCHP_CORESPI_INT_TXDONE) {
		spiwrite8(cfg, MCHP_CORESPI_INT_TXDONE, MCHP_CORESPI_REG_INTCLEAR);
	}
	if (intfield & MCHP_CORESPI_INT_RX_CHANNEL_OVERFLOW) {
		spiwrite8(cfg, MCHP_CORESPI_INT_RX_CHANNEL_OVERFLOW, MCHP_CORESPI_REG_INTCLEAR);
		LOG_ERR("RX OVERFLOW");
	}
	if (intfield & MCHP_CORESPI_INT_TX_CHANNEL_UNDERRUN) {
		spiwrite8(cfg, MCHP_CORESPI_INT_TX_CHANNEL_UNDERRUN, MCHP_CORESPI_REG_INTCLEAR);
		LOG_ERR("TX UNDERRUN");
	}
}

static int corespi_init(const struct device *dev)
{
	const struct mchp_corespi_config *cfg = dev->config;
	struct mchp_corespi_data *data = dev->data;

	cfg->irq_config_func(dev);

	/* Put core in master + enable */
	uint8_t c = spiread8(cfg, MCHP_CORESPI_REG_CONTROL);
	c = (c & ~MCHP_CORESPI_CONTROL_ENABLE) | MCHP_CORESPI_CONTROL_MASTER;
	spiwrite8(cfg, c, MCHP_CORESPI_REG_CONTROL);
	corespi_enable_ints(cfg);
	corespi_enable(cfg);

	spi_context_unlock_unconditionally(&data->ctx);
	return 0;
}

static DEVICE_API(spi, corespi_api) = {
	.transceive = corespi_transceive_blocking,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = corespi_transceive_async,
#endif
	.release = corespi_release,
};

#define CORESPI_INIT(inst) \
	static void corespi_irq_cfg_##inst(const struct device *dev) \
	{ \
		IRQ_CONNECT(DT_INST_IRQN(inst), \
			    DT_INST_IRQ(inst, priority), \
			    corespi_isr, \
			    DEVICE_DT_INST_GET(inst), 0); \
		irq_enable(DT_INST_IRQN(inst)); \
	} \
	\
	static const struct mchp_corespi_config corespi_cfg_##inst = { \
		.base = DT_INST_REG_ADDR(inst), \
		.irq_config_func = corespi_irq_cfg_##inst, \
		.clock_freq = DT_INST_PROP(inst, clock_frequency), \
		.fifo_depth = DT_INST_PROP_OR(inst, fifo_depth, MCHP_CORESPI_DEFAULT_FIFO_DEPTH), \
		.motorola_mode = DT_INST_PROP_OR(inst, motorola_mode, MCHP_CORESPI_DEFAULT_MOTOROLA_MODE), \
		.use_internal_ssel = DT_INST_PROP_OR(inst, ssel_active, true), \
	}; \
	\
	static struct mchp_corespi_data corespi_data_##inst = { \
		SPI_CONTEXT_INIT_LOCK(corespi_data_##inst, ctx), \
		SPI_CONTEXT_INIT_SYNC(corespi_data_##inst, ctx), \
	}; \
	\
	SPI_DEVICE_DT_INST_DEFINE(inst, \
				  corespi_init, \
				  NULL, \
				  &corespi_data_##inst, \
				  &corespi_cfg_##inst, \
				  POST_KERNEL, \
				  CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
				  &corespi_api);

DT_INST_FOREACH_STATUS_OKAY(CORESPI_INIT)

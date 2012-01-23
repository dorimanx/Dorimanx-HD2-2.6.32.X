/*
 * soc-cache.c  --  ASoC register cache helpers
 *
 * Copyright 2009 Wolfson Microelectronics PLC.
 *
 * Author: Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <sound/soc.h>

static unsigned int snd_soc_7_9_read(struct snd_soc_codec *codec,
				     unsigned int reg)
{
	u16 *cache = codec->reg_cache;
	if (reg >= codec->reg_cache_size)
		return -1;
	return cache[reg];
}

static int snd_soc_7_9_write(struct snd_soc_codec *codec, unsigned int reg,
			     unsigned int value)
{
	u16 *cache = codec->reg_cache;
	u8 data[2];
	int ret;

	BUG_ON(codec->volatile_register);

	data[0] = (reg << 1) | ((value >> 8) & 0x0001);
	data[1] = value & 0x00ff;

	if (reg < codec->reg_cache_size)
		cache[reg] = value;
	ret = codec->hw_write(codec->control_data, data, 2);
	if (ret == 2)
		return 0;
	if (ret < 0)
		return ret;
	else
		return -EIO;
}

#if defined(CONFIG_SPI_MASTER)
static int snd_soc_7_9_spi_write(void *control_data, const char *data,
				 int len)
{
	struct spi_device *spi = control_data;
	struct spi_transfer t;
	struct spi_message m;
	u8 msg[2];

	if (len <= 0)
		return 0;

	msg[0] = data[0];
	msg[1] = data[1];

	spi_message_init(&m);
	memset(&t, 0, (sizeof t));

	t.tx_buf = &msg[0];
	t.len = len;

	spi_message_add_tail(&t, &m);
	spi_sync(spi, &m);

	return len;
}
#else
#define snd_soc_7_9_spi_write NULL
#endif

static int snd_soc_8_16_write(struct snd_soc_codec *codec, unsigned int reg,
			      unsigned int value)
{
	u16 *reg_cache = codec->reg_cache;
	u8 data[3];

	data[0] = reg;
	data[1] = (value >> 8) & 0xff;
	data[2] = value & 0xff;

	if (!snd_soc_codec_volatile_register(codec, reg))
		reg_cache[reg] = value;

	if (codec->hw_write(codec->control_data, data, 3) == 3)
		return 0;
	else
		return -EIO;
}

static unsigned int snd_soc_8_16_read(struct snd_soc_codec *codec,
				      unsigned int reg)
{
	u16 *cache = codec->reg_cache;

	if (reg >= codec->reg_cache_size ||
	    snd_soc_codec_volatile_register(codec, reg))
		return codec->hw_read(codec, reg);
	else
		return cache[reg];
}

#if defined(CONFIG_I2C) || (defined(CONFIG_I2C_MODULE) && defined(MODULE))
static unsigned int snd_soc_8_16_read_i2c(struct snd_soc_codec *codec,
					  unsigned int r)
{
	struct i2c_msg xfer[2];
	u8 reg = r;
	u16 data;
	int ret;
	struct i2c_client *client = codec->control_data;

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 1;
	xfer[0].buf = &reg;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = 2;
	xfer[1].buf = (u8 *)&data;

	ret = i2c_transfer(client->adapter, xfer, 2);
	if (ret != 2) {
		dev_err(&client->dev, "i2c_transfer() returned %d\n", ret);
		return 0;
	}

	return (data >> 8) | ((data & 0xff) << 8);
}
#else
#define snd_soc_8_16_read_i2c NULL
#endif

static struct {
	int addr_bits;
	int data_bits;
	int (*write)(struct snd_soc_codec *codec, unsigned int, unsigned int);
	int (*spi_write)(void *, const char *, int);
	unsigned int (*read)(struct snd_soc_codec *, unsigned int);
	unsigned int (*i2c_read)(struct snd_soc_codec *, unsigned int);
} io_types[] = {
	{ 7, 9, snd_soc_7_9_write, snd_soc_7_9_spi_write, snd_soc_7_9_read },
	{ 8, 16, snd_soc_8_16_write, NULL, snd_soc_8_16_read,
	  snd_soc_8_16_read_i2c },
};

/**
 * snd_soc_codec_set_cache_io: Set up standard I/O functions.
 *
 * @codec: CODEC to configure.
 * @type: Type of cache.
 * @addr_bits: Number of bits of register address data.
 * @data_bits: Number of bits of data per register.
 * @control: Control bus used.
 *
 * Register formats are frequently shared between many I2C and SPI
 * devices.  In order to promote code reuse the ASoC core provides
 * some standard implementations of CODEC read and write operations
 * which can be set up using this function.
 *
 * The caller is responsible for allocating and initialising the
 * actual cache.
 *
 * Note that at present this code cannot be used by CODECs with
 * volatile registers.
 */
int snd_soc_codec_set_cache_io(struct snd_soc_codec *codec,
			       int addr_bits, int data_bits,
			       enum snd_soc_control_type control)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(io_types); i++)
		if (io_types[i].addr_bits == addr_bits &&
		    io_types[i].data_bits == data_bits)
			break;
	if (i == ARRAY_SIZE(io_types)) {
		printk(KERN_ERR
		       "No I/O functions for %d bit address %d bit data\n",
		       addr_bits, data_bits);
		return -EINVAL;
	}

	codec->write = io_types[i].write;
	codec->read = io_types[i].read;

	switch (control) {
	case SND_SOC_CUSTOM:
		break;

	case SND_SOC_I2C:
#if defined(CONFIG_I2C) || (defined(CONFIG_I2C_MODULE) && defined(MODULE))
		codec->hw_write = (hw_write_t)i2c_master_send;
#endif
		if (io_types[i].i2c_read)
			codec->hw_read = io_types[i].i2c_read;
		break;

	case SND_SOC_SPI:
		if (io_types[i].spi_write)
			codec->hw_write = io_types[i].spi_write;
		break;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_codec_set_cache_io);

#ifdef CONFIG_SND_SOC_CACHE_LZO
struct snd_soc_lzo_ctx {
	void *wmem;
	void *dst;
	const void *src;
	size_t src_len;
	size_t dst_len;
	size_t decompressed_size;
	unsigned long *sync_bmp;
	int sync_bmp_nbits;
};

#define LZO_BLOCK_NUM 8
static int snd_soc_lzo_block_count(void)
{
	return LZO_BLOCK_NUM;
}

static int snd_soc_lzo_prepare(struct snd_soc_lzo_ctx *lzo_ctx)
{
	lzo_ctx->wmem = kmalloc(LZO1X_MEM_COMPRESS, GFP_KERNEL);
	if (!lzo_ctx->wmem)
		return -ENOMEM;
	return 0;
}

static int snd_soc_lzo_compress(struct snd_soc_lzo_ctx *lzo_ctx)
{
	size_t compress_size;
	int ret;

	ret = lzo1x_1_compress(lzo_ctx->src, lzo_ctx->src_len,
			       lzo_ctx->dst, &compress_size, lzo_ctx->wmem);
	if (ret != LZO_E_OK || compress_size > lzo_ctx->dst_len)
		return -EINVAL;
	lzo_ctx->dst_len = compress_size;
	return 0;
}

static int snd_soc_lzo_decompress(struct snd_soc_lzo_ctx *lzo_ctx)
{
	size_t dst_len;
	int ret;

	dst_len = lzo_ctx->dst_len;
	ret = lzo1x_decompress_safe(lzo_ctx->src, lzo_ctx->src_len,
				    lzo_ctx->dst, &dst_len);
	if (ret != LZO_E_OK || dst_len != lzo_ctx->dst_len)
		return -EINVAL;
	return 0;
}

static int snd_soc_lzo_compress_cache_block(struct snd_soc_codec *codec,
		struct snd_soc_lzo_ctx *lzo_ctx)
{
	int ret;

	lzo_ctx->dst_len = lzo1x_worst_compress(PAGE_SIZE);
	lzo_ctx->dst = kmalloc(lzo_ctx->dst_len, GFP_KERNEL);
	if (!lzo_ctx->dst) {
		lzo_ctx->dst_len = 0;
		return -ENOMEM;
	}

	ret = snd_soc_lzo_compress(lzo_ctx);
	if (ret < 0)
		return ret;
	return 0;
}

static int snd_soc_lzo_decompress_cache_block(struct snd_soc_codec *codec,
		struct snd_soc_lzo_ctx *lzo_ctx)
{
	int ret;

	lzo_ctx->dst_len = lzo_ctx->decompressed_size;
	lzo_ctx->dst = kmalloc(lzo_ctx->dst_len, GFP_KERNEL);
	if (!lzo_ctx->dst) {
		lzo_ctx->dst_len = 0;
		return -ENOMEM;
	}

	ret = snd_soc_lzo_decompress(lzo_ctx);
	if (ret < 0)
		return ret;
	return 0;
}

static inline int snd_soc_lzo_get_blkindex(struct snd_soc_codec *codec,
		unsigned int reg)
{
	const struct snd_soc_codec_driver *codec_drv;

	codec_drv = codec->driver;
	return (reg * codec_drv->reg_word_size) /
	       DIV_ROUND_UP(codec->reg_size, snd_soc_lzo_block_count());
}

static inline int snd_soc_lzo_get_blkpos(struct snd_soc_codec *codec,
		unsigned int reg)
{
	const struct snd_soc_codec_driver *codec_drv;

	codec_drv = codec->driver;
	return reg % (DIV_ROUND_UP(codec->reg_size, snd_soc_lzo_block_count()) /
		      codec_drv->reg_word_size);
}

static inline int snd_soc_lzo_get_blksize(struct snd_soc_codec *codec)
{
	return DIV_ROUND_UP(codec->reg_size, snd_soc_lzo_block_count());
}

static int snd_soc_lzo_cache_sync(struct snd_soc_codec *codec)
{
	struct snd_soc_lzo_ctx **lzo_blocks;
	unsigned int val;
	int i;
	int ret;

	lzo_blocks = codec->reg_cache;
	for_each_set_bit(i, lzo_blocks[0]->sync_bmp, lzo_blocks[0]->sync_bmp_nbits) {
		WARN_ON(!snd_soc_codec_writable_register(codec, i));
		ret = snd_soc_cache_read(codec, i, &val);
		if (ret)
			return ret;
		codec->cache_bypass = 1;
		ret = snd_soc_write(codec, i, val);
		codec->cache_bypass = 0;
		if (ret)
			return ret;
		dev_dbg(codec->dev, "Synced register %#x, value = %#x\n",
			i, val);
	}

	return 0;
}

static int snd_soc_lzo_cache_write(struct snd_soc_codec *codec,
				   unsigned int reg, unsigned int value)
{
	struct snd_soc_lzo_ctx *lzo_block, **lzo_blocks;
	int ret, blkindex, blkpos;
	size_t blksize, tmp_dst_len;
	void *tmp_dst;

	/* index of the compressed lzo block */
	blkindex = snd_soc_lzo_get_blkindex(codec, reg);
	/* register index within the decompressed block */
	blkpos = snd_soc_lzo_get_blkpos(codec, reg);
	/* size of the compressed block */
	blksize = snd_soc_lzo_get_blksize(codec);
	lzo_blocks = codec->reg_cache;
	lzo_block = lzo_blocks[blkindex];

	/* save the pointer and length of the compressed block */
	tmp_dst = lzo_block->dst;
	tmp_dst_len = lzo_block->dst_len;

	/* prepare the source to be the compressed block */
	lzo_block->src = lzo_block->dst;
	lzo_block->src_len = lzo_block->dst_len;

	/* decompress the block */
	ret = snd_soc_lzo_decompress_cache_block(codec, lzo_block);
	if (ret < 0) {
		kfree(lzo_block->dst);
		goto out;
	}

	/* write the new value to the cache */
	if (snd_soc_set_cache_val(lzo_block->dst, blkpos, value,
				  codec->driver->reg_word_size)) {
		kfree(lzo_block->dst);
		goto out;
	}

	/* prepare the source to be the decompressed block */
	lzo_block->src = lzo_block->dst;
	lzo_block->src_len = lzo_block->dst_len;

	/* compress the block */
	ret = snd_soc_lzo_compress_cache_block(codec, lzo_block);
	if (ret < 0) {
		kfree(lzo_block->dst);
		kfree(lzo_block->src);
		goto out;
	}

	/* set the bit so we know we have to sync this register */
	set_bit(reg, lzo_block->sync_bmp);
	kfree(tmp_dst);
	kfree(lzo_block->src);
	return 0;
out:
	lzo_block->dst = tmp_dst;
	lzo_block->dst_len = tmp_dst_len;
	return ret;
}

static int snd_soc_lzo_cache_read(struct snd_soc_codec *codec,
				  unsigned int reg, unsigned int *value)
{
	struct snd_soc_lzo_ctx *lzo_block, **lzo_blocks;
	int ret, blkindex, blkpos;
	size_t blksize, tmp_dst_len;
	void *tmp_dst;

	*value = 0;
	/* index of the compressed lzo block */
	blkindex = snd_soc_lzo_get_blkindex(codec, reg);
	/* register index within the decompressed block */
	blkpos = snd_soc_lzo_get_blkpos(codec, reg);
	/* size of the compressed block */
	blksize = snd_soc_lzo_get_blksize(codec);
	lzo_blocks = codec->reg_cache;
	lzo_block = lzo_blocks[blkindex];

	/* save the pointer and length of the compressed block */
	tmp_dst = lzo_block->dst;
	tmp_dst_len = lzo_block->dst_len;

	/* prepare the source to be the compressed block */
	lzo_block->src = lzo_block->dst;
	lzo_block->src_len = lzo_block->dst_len;

	/* decompress the block */
	ret = snd_soc_lzo_decompress_cache_block(codec, lzo_block);
	if (ret >= 0)
		/* fetch the value from the cache */
		*value = snd_soc_get_cache_val(lzo_block->dst, blkpos,
					       codec->driver->reg_word_size);

	kfree(lzo_block->dst);
	/* restore the pointer and length of the compressed block */
	lzo_block->dst = tmp_dst;
	lzo_block->dst_len = tmp_dst_len;
	return 0;
}

static int snd_soc_lzo_cache_exit(struct snd_soc_codec *codec)
{
	struct snd_soc_lzo_ctx **lzo_blocks;
	int i, blkcount;

	lzo_blocks = codec->reg_cache;
	if (!lzo_blocks)
		return 0;

	blkcount = snd_soc_lzo_block_count();
	/*
	 * the pointer to the bitmap used for syncing the cache
	 * is shared amongst all lzo_blocks.  Ensure it is freed
	 * only once.
	 */
	if (lzo_blocks[0])
		kfree(lzo_blocks[0]->sync_bmp);
	for (i = 0; i < blkcount; ++i) {
		if (lzo_blocks[i]) {
			kfree(lzo_blocks[i]->wmem);
			kfree(lzo_blocks[i]->dst);
		}
		/* each lzo_block is a pointer returned by kmalloc or NULL */
		kfree(lzo_blocks[i]);
	}
	kfree(lzo_blocks);
	codec->reg_cache = NULL;
	return 0;
}

static int snd_soc_lzo_cache_init(struct snd_soc_codec *codec)
{
	struct snd_soc_lzo_ctx **lzo_blocks;
	size_t bmp_size;
	const struct snd_soc_codec_driver *codec_drv;
	int ret, tofree, i, blksize, blkcount;
	const char *p, *end;
	unsigned long *sync_bmp;

	ret = 0;
	codec_drv = codec->driver;

	/*
	 * If we have not been given a default register cache
	 * then allocate a dummy zero-ed out region, compress it
	 * and remember to free it afterwards.
	 */
	tofree = 0;
	if (!codec->reg_def_copy)
		tofree = 1;

	if (!codec->reg_def_copy) {
		codec->reg_def_copy = kzalloc(codec->reg_size, GFP_KERNEL);
		if (!codec->reg_def_copy)
			return -ENOMEM;
	}

	blkcount = snd_soc_lzo_block_count();
	codec->reg_cache = kzalloc(blkcount * sizeof *lzo_blocks,
				   GFP_KERNEL);
	if (!codec->reg_cache) {
		ret = -ENOMEM;
		goto err_tofree;
	}
	lzo_blocks = codec->reg_cache;

	/*
	 * allocate a bitmap to be used when syncing the cache with
	 * the hardware.  Each time a register is modified, the corresponding
	 * bit is set in the bitmap, so we know that we have to sync
	 * that register.
	 */
	bmp_size = codec_drv->reg_cache_size;
	sync_bmp = kmalloc(BITS_TO_LONGS(bmp_size) * sizeof(long),
			   GFP_KERNEL);
	if (!sync_bmp) {
		ret = -ENOMEM;
		goto err;
	}
	bitmap_zero(sync_bmp, bmp_size);

	/* allocate the lzo blocks and initialize them */
	for (i = 0; i < blkcount; ++i) {
		lzo_blocks[i] = kzalloc(sizeof **lzo_blocks,
					GFP_KERNEL);
		if (!lzo_blocks[i]) {
			kfree(sync_bmp);
			ret = -ENOMEM;
			goto err;
		}
		lzo_blocks[i]->sync_bmp = sync_bmp;
		lzo_blocks[i]->sync_bmp_nbits = bmp_size;
		/* alloc the working space for the compressed block */
		ret = snd_soc_lzo_prepare(lzo_blocks[i]);
		if (ret < 0)
			goto err;
	}

	blksize = snd_soc_lzo_get_blksize(codec);
	p = codec->reg_def_copy;
	end = codec->reg_def_copy + codec->reg_size;
	/* compress the register map and fill the lzo blocks */
	for (i = 0; i < blkcount; ++i, p += blksize) {
		lzo_blocks[i]->src = p;
		if (p + blksize > end)
			lzo_blocks[i]->src_len = end - p;
		else
			lzo_blocks[i]->src_len = blksize;
		ret = snd_soc_lzo_compress_cache_block(codec,
						       lzo_blocks[i]);
		if (ret < 0)
			goto err;
		lzo_blocks[i]->decompressed_size =
			lzo_blocks[i]->src_len;
	}

	if (tofree) {
		kfree(codec->reg_def_copy);
		codec->reg_def_copy = NULL;
	}
	return 0;
err:
	snd_soc_cache_exit(codec);
err_tofree:
	if (tofree) {
		kfree(codec->reg_def_copy);
		codec->reg_def_copy = NULL;
	}
	return ret;
}
#endif

static int snd_soc_flat_cache_sync(struct snd_soc_codec *codec)
{
	int i;
	int ret;
	const struct snd_soc_codec_driver *codec_drv;
	unsigned int val;

	codec_drv = codec->driver;
	for (i = 0; i < codec_drv->reg_cache_size; ++i) {
		ret = snd_soc_cache_read(codec, i, &val);
		if (ret)
			return ret;
		if (codec->reg_def_copy)
			if (snd_soc_get_cache_val(codec->reg_def_copy,
						  i, codec_drv->reg_word_size) == val)
				continue;

		WARN_ON(!snd_soc_codec_writable_register(codec, i));

		ret = snd_soc_write(codec, i, val);
		if (ret)
			return ret;
		dev_dbg(codec->dev, "Synced register %#x, value = %#x\n",
			i, val);
	}
	return 0;
}

static int snd_soc_flat_cache_write(struct snd_soc_codec *codec,
				    unsigned int reg, unsigned int value)
{
	snd_soc_set_cache_val(codec->reg_cache, reg, value,
			      codec->driver->reg_word_size);
	return 0;
}

static int snd_soc_flat_cache_read(struct snd_soc_codec *codec,
				   unsigned int reg, unsigned int *value)
{
	*value = snd_soc_get_cache_val(codec->reg_cache, reg,
				       codec->driver->reg_word_size);
	return 0;
}

static int snd_soc_flat_cache_exit(struct snd_soc_codec *codec)
{
	if (!codec->reg_cache)
		return 0;
	kfree(codec->reg_cache);
	codec->reg_cache = NULL;
	return 0;
}

static int snd_soc_flat_cache_init(struct snd_soc_codec *codec)
{
	if (codec->reg_def_copy)
		codec->reg_cache = kmemdup(codec->reg_def_copy,
					   codec->reg_size, GFP_KERNEL);
	else
		codec->reg_cache = kzalloc(codec->reg_size, GFP_KERNEL);
	if (!codec->reg_cache)
		return -ENOMEM;

	return 0;
}

/* an array of all supported compression types */
static const struct snd_soc_cache_ops cache_types[] = {
	/* Flat *must* be the first entry for fallback */
	{
		.id = SND_SOC_FLAT_COMPRESSION,
		.name = "flat",
		.init = snd_soc_flat_cache_init,
		.exit = snd_soc_flat_cache_exit,
		.read = snd_soc_flat_cache_read,
		.write = snd_soc_flat_cache_write,
		.sync = snd_soc_flat_cache_sync
	},
#ifdef CONFIG_SND_SOC_CACHE_LZO
	{
		.id = SND_SOC_LZO_COMPRESSION,
		.name = "LZO",
		.init = snd_soc_lzo_cache_init,
		.exit = snd_soc_lzo_cache_exit,
		.read = snd_soc_lzo_cache_read,
		.write = snd_soc_lzo_cache_write,
		.sync = snd_soc_lzo_cache_sync
	},
#endif
	{
		.id = SND_SOC_RBTREE_COMPRESSION,
		.name = "rbtree",
		.init = snd_soc_rbtree_cache_init,
		.exit = snd_soc_rbtree_cache_exit,
		.read = snd_soc_rbtree_cache_read,
		.write = snd_soc_rbtree_cache_write,
		.sync = snd_soc_rbtree_cache_sync
	}
};

int snd_soc_cache_init(struct snd_soc_codec *codec)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cache_types); ++i)
		if (cache_types[i].id == codec->compress_type)
			break;

	/* Fall back to flat compression */
	if (i == ARRAY_SIZE(cache_types)) {
		dev_warn(codec->dev, "Could not match compress type: %d\n",
			 codec->compress_type);
		i = 0;
	}

	mutex_init(&codec->cache_rw_mutex);
	codec->cache_ops = &cache_types[i];

	if (codec->cache_ops->init) {
		if (codec->cache_ops->name)
			dev_dbg(codec->dev, "Initializing %s cache for %s codec\n",
				codec->cache_ops->name, codec->name);
		return codec->cache_ops->init(codec);
	}
	return -ENOSYS;
}

/*
 * NOTE: keep in mind that this function might be called
 * multiple times.
 */
int snd_soc_cache_exit(struct snd_soc_codec *codec)
{
	if (codec->cache_ops && codec->cache_ops->exit) {
		if (codec->cache_ops->name)
			dev_dbg(codec->dev, "Destroying %s cache for %s codec\n",
				codec->cache_ops->name, codec->name);
		return codec->cache_ops->exit(codec);
	}
	return -ENOSYS;
}

/**
 * snd_soc_cache_read: Fetch the value of a given register from the cache.
 *
 * @codec: CODEC to configure.
 * @reg: The register index.
 * @value: The value to be returned.
 */
int snd_soc_cache_read(struct snd_soc_codec *codec,
		       unsigned int reg, unsigned int *value)
{
	int ret;

	mutex_lock(&codec->cache_rw_mutex);

	if (value && codec->cache_ops && codec->cache_ops->read) {
		ret = codec->cache_ops->read(codec, reg, value);
		mutex_unlock(&codec->cache_rw_mutex);
		return ret;
	}

	mutex_unlock(&codec->cache_rw_mutex);
	return -ENOSYS;
}


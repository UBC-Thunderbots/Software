#include "software/jetson_nano/encoder/encoder_spi.h"
#include <endian.h>
#include <fcntl.h>
#include "software/logger/logger.h"

EncoderSpi::EncoderSpi(uint8_t chip_select_pin)
	: chipSelectPin(chip_select_pin)
{
	fd = open("/dev/spidev1.0", O_RDWR); 
	CHECK(fd >= 0) << "can't open encoder, error: " << strerror(errno);	

//	int ret = ioctl(fd, SPI_IOC_WR_MODE32, SPI_MODE_1);
//	CHECK(ret != -1) << "can't set spi mode for encoder, error: " << strerror(errno);
}

void EncoderSpi::writeData(uint16_t command, uint16_t value) {
	int ret;	

	struct spi_ioc_transfer tr[2];
	memset(tr, 0, sizeof(tr));

	tr[0].tx_buf 		= command;
	tr[0].rx_buf 		= 0;
	tr[0].len 		= 16;
	tr[0].delay_usecs 	= 0;
	tr[0].speed_hz 		= SPI_SPEED_HZ;
	tr[0].bits_per_word = 16;

	tr[1].tx_buf 		= value;
	tr[1].rx_buf 	 	= 0;
	tr[1].len 	 	= 16;
	tr[1].delay_usecs 	= 0;
	tr[1].speed_hz 		= SPI_SPEED_HZ;
	tr[1].bits_per_word 	= 16;
	
	ret = ioctl(fd, SPI_IOC_MESSAGE(2), &tr);

	CHECK(ret >= 1) << "SPI Transfer to encoder failed, errno " << strerror(errno);

	return;
}

uint16_t EncoderSpi::readData(uint16_t command, uint16_t nopCommand) {
	int ret;

	struct spi_ioc_transfer tr[2];
	memset(tr, 0, sizeof(tr));

	unsigned char spi_mode = SPI_MODE_1;
	ret = ioctl(fd, SPI_IOC_WR_MODE32, &spi_mode);
	CHECK(ret != -1) << "can't set spi mode for encoder, error: " << strerror(errno);

	tr[0].tx_buf 		= (unsigned long) tx;
	tr[0].rx_buf 		= (unsigned long) rx;
	tr[0].len 		= 16;
	tr[0].delay_usecs 	= 0;
	tr[0].speed_hz 		= SPI_SPEED_HZ;
	tr[0].bits_per_word 	= 16;

	tr[1].tx_buf 		= (unsigned long) tx;
	tr[1].rx_buf	 	= (unsigned long) rx;
	tr[1].len 		= 16;
	tr[1].delay_usecs  	= 0;
	tr[1].speed_hz 		= SPI_SPEED_HZ;
	tr[1].bits_per_word 	= 16;

	ret = ioctl(fd, SPI_IOC_MESSAGE(2), &tr);

	CHECK(ret >= 1) << "SPI transfer to encoder failed, errno " << strerror(errno);

	return rx[0];
}

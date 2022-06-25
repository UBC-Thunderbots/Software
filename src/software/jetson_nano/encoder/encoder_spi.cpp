#include "software/jetson_nano/encoder/encoder_spi.h"
#include <endian.h>
#include <fcntl.h>
#include "software/logger/logger.h"

EncoderSpi::EncoderSpi(uint8_t chip_select_pin)
	: chipSelectPin(chip_select_pin)
{
	LOG(DEBUG) << "open spi port";
	int ret;
	fd = open("/dev/spidev0.0", O_RDWR); 
	CHECK(fd >= 0) << "can't open encoder, error: " << strerror(errno);	

//	int ret = ioctl(fd, SPI_IOC_WR_MODE32, SPI_MODE_1);
//	CHECK(ret != -1) << "can't set spi mode for encoder, error: " << strerror(errno);
	LOG(DEBUG) << "open spi mode";
       unsigned char spi_mode = SPI_MODE_1;
          ret = ioctl(fd, SPI_IOC_WR_MODE, &spi_mode);
          CHECK(ret != -1) << "can't set spi mode for encoder, error: " << strerror(errno);	
	  LOG(DEBUG) << "finished setting spi mode";
	  LOG(DEBUG) << "set spi speed";
	  uint32_t speed_ = 10000;
	  ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_);
	  CHECK(ret != -1) << "cant set spi speed for encoder, error: " << strerror(errno);
	  LOG(DEBUG) << "finsihed setting spi speed";
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

	tr[1].tx_buf 		= value;
	tr[1].rx_buf 	 	= 0;
	tr[1].len 	 	= 16;
	tr[1].delay_usecs 	= 0;
	tr[1].speed_hz 		= SPI_SPEED_HZ;
	
	ret = ioctl(fd, SPI_IOC_MESSAGE(2), &tr);

	CHECK(ret >= 1) << "SPI Transfer to encoder failed, errno " << strerror(errno);

	return;
}

uint16_t EncoderSpi::readData(uint16_t command, uint16_t nopCommand) {
	int ret;

	struct spi_ioc_transfer tr[2];
	memset(tr, 0, sizeof(tr));
	memset(tx, 0, sizeof(tx));
	memset(rx, 0, sizeof(rx));

	unsigned char spi_mode = SPI_MODE_1;
	ret = ioctl(fd, SPI_IOC_RD_MODE, &spi_mode);
	CHECK(ret != -1) << "can't set spi mode for encoder, error: " << strerror(errno);

	tr[0].tx_buf 		= (unsigned long) &command;
	tr[0].rx_buf 		= (unsigned long) &rx[0];
	tr[0].len 		= 4;
	tr[0].delay_usecs 	= 0;
	tr[0].speed_hz 		= SPI_SPEED_HZ;

	tr[1].tx_buf 		= (unsigned long) &nopCommand;
	tr[1].rx_buf	 	= (unsigned long) &rx[1];
	tr[1].len 		= 4;
	tr[1].delay_usecs  	= 0;
	tr[1].speed_hz 		= SPI_SPEED_HZ;

	ret = ioctl(fd, SPI_IOC_MESSAGE(2), &tr);

	CHECK(ret >= 1) << "SPI transfer to encoder failed, errno " << strerror(errno);

	for (int i = 0;i < 4; ++i)
	{
		std::cout << rx[i] << "\n";	
	}
	return rx[1];
}

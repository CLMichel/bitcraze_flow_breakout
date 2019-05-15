#include <stdint.h>
#include <PMW3901/bcm2835.h>

class PMW3901{
	private :
	void registerWrite(uint8_t reg, uint8_t value)const;
	uint8_t registerRead(uint8_t reg)const;
	void initRegisters(void)const;
	uint8_t m_cs;

	public :
	PMW3901(uint8_t cs);
	int begin(void)const;
	void readMotionCount(int16_t *deltaX, int16_t *deltaY)const;


};

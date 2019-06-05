#include <stdint.h>
#include <PMW3901/bcm2835.h>

class PMW3901{
	private :
	//write register reg with value
	void registerWrite(uint8_t reg, uint8_t value)const;
	//read register reg
	uint8_t registerRead(uint8_t reg)const;
	//initialization of registers
	void initRegisters(void)const;
	uint8_t m_cs;

	public :
	//constructor
	PMW3901(uint8_t cs);
	//initialization of sensor
	int begin(void)const;
	//read delta X and delta Y in sensor's register
	void readMotionCount(int16_t *deltaX, int16_t *deltaY)const;


};

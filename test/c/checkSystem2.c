#include <stdint.h>

#define ADDR_LEDS   ((uint8_t *) 0x00008000)
#define ADDR_SWITCHES ((volatile uint8_t *) 0x00008010)
#define ADDR_ALLHIGH ((volatile uint32_t *) 0x00008020)

void light()
{
	static uint32_t cnt = 0xFF;
	*ADDR_LEDS = *ADDR_SWITCHES;
}

int main(){
    int i = 0;
	uint8_t cnt = 255;
	
    while(1){
        if (i == 100000) {
			light();
			i = 0;
        } else {
            i++;
        }
    }
    return 0;
}

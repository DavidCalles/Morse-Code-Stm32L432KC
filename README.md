# Morse Code Stm32L432KC
Output morse code of a given string through the GPIO.
The code is presented under the folder arrengement of a CubeIDE V1.6.0 project and is tested on a Stm32L432KC board.

## Features
- String to morse conversion with GPIO output.
- Serial terminal verbosity.
- 12x6 LCD output.

## Dependencies
- HAL libraries for STM-32L4XX.
- Adapted external library for LCD management.

## Usage
The code outputs the morse code through a GPIO output. 
By default is pin D2 of an Stm32L432KC board. A minimum of a resistor and a LED are recommended.
The input string to be converted is fixed and can be changed in the main function:

    char myname[] = "David Calles\n"; // Input string to convert
    
The output GPIO port and pin can be changed in:

    /*LED pin*/
    #define LED_GPIO GPIOA		// Output GPIO
    #define LED_PIN GPIO_PIN_12	// Output pin (Pin D2 on board)
To Disable the serial port output and/or the LCD output, change to 0:

    /*Serial Output*/
    #define SERIAL_ENABLE 1		// Enable verbose through serial terminal
    							// Disable=0
    /*LCD*/
    #define LCD_ENABLE 1		// Enable LCD output, Disable=0
The LCD output is configured in 8-bit mode, and the ports and pins used are the following:

    /*LCD screen pins*/
    #define D0_GPIO_Port GPIOB // Data 0 GPIO port
    #define D1_GPIO_Port GPIOB // Data 1 GPIO port
    #define D2_GPIO_Port GPIOB // Data 2 GPIO port
    #define D3_GPIO_Port GPIOB // Data 3 GPIO port
    #define D4_GPIO_Port GPIOA // Data 4 GPIO port
    #define D5_GPIO_Port GPIOA // Data 5 GPIO port
    #define D6_GPIO_Port GPIOB // Data 6 GPIO port
    #define D7_GPIO_Port GPIOB // Data 7 GPIO port
    
    #define D0_Pin GPIO_PIN_0 // Data 0 pin (D3 on board)
    #define D1_Pin GPIO_PIN_7 // Data 0 pin (D4 on board)
    #define D2_Pin GPIO_PIN_6 // Data 0 pin (D5 on board)
    #define D3_Pin GPIO_PIN_1 // Data 0 pin (D6 on board)
    #define D4_Pin GPIO_PIN_8 // Data 0 pin (D9 on board)
    #define D5_Pin GPIO_PIN_11// Data 0 pin (D10 on board)
    #define D6_Pin GPIO_PIN_5 // Data 0 pin (D11 on board)
    #define D7_Pin GPIO_PIN_4 // Data 0 pin (D12 on board)
    
    #define RS_GPIO_Port GPIOA // Register select GPIO port
    #define RS_Pin GPIO_PIN_9  // Register select pin (D1)
    #define EN_GPIO_Port GPIOA // Enable GPIO port
    #define EN_Pin GPIO_PIN_10 // Register select pin (D0)
Please note that the provided code manually sets all this pins as outputs and if changed, the new pins should be configured as outputs too.

## Improvements
 Code is far from perfect. 
 If you would like to propose a better implementation or add any features feel free to post a pull request.

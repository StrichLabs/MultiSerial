## MultiSerial()

### Description
Creates a variable of type MultiSerial, to communciate with the MultiSerial Shield.

### Syntax
MultiSerial(*address*, *port*)

### Parameters
*address* - Address on the I2C bus of the shield to communicate with
*port* - Which port (0 or 1) on the shield to associate this instance with

### Returns
Instance of MultiSerial associated with a specific port on a specific shield. 

### Example
    #include <MultiSerial.h>
    
    // Create a MultiSerial instance to communicate with port 0 on shield 0x4D
    MultiSerial msInstance = MultiSerial(0x4D, 0);             

## begin()

### Description
Sets the data rate in bits per second for serial data transmission.

### Syntax
*MultiSerialInstance*.begin(*speed*)

### Parameters
*speed* - communications speed in bits per second

### Returns
nothing

### Example
    #include <MultiSerial.h>
    
    // Create a MultiSerial instance to communicate with port 0 on shield 0x4D
    MultiSerial msInstance = MultiSerial(0x4D, 0);
    
    void setup() {
      // Set up port for communications at 9600 bits per second
      msInstance.begin(9600);
    }
## write()

### Description
Transmit a byte of data out the serial port.

### Syntax
*MultiSerialInstance*.write(*data*)

### Parameters
*data* - byte of data to write

### Returns
nothing

### Example
    #include <MultiSerial.h>
    
    // Create a MultiSerial instance to communicate with port 0 on shield 0x4D
    MultiSerial msInstance = MultiSerial(0x4D, 0);
    
    void setup() {
      // Set up port for communications at 9600 bits per second
      msInstance.begin(9600);
    
      // Send 42 out the serial port initialized above
      msInstance.write(42);
    }

## read()

### Description
Read the next available byte of available serial data.

### Syntax
*MultiSerialInstance*.read(**)

### Parameters
*none

### Returns
first available byte of serial data, or -1 if no data is available

### Example
    #include <MultiSerial.h>
    
    // Create a MultiSerial instance to communicate with port 0 on shield 0x4D
    MultiSerial msInstance = MultiSerial(0x4D, 0);
    
    void setup() {
      // Set up port for communications at 9600 bits per second
      msInstance.begin(9600);
    }
    
    void loop () {
      byte data;
      if(msInstance.available() > 0) {
        // read in whatever the next byte of available data is and store it in data
        data = msInstance.read()
      }
    }
## available()

### Description
Get the number of bytes available for reading from the serial port.

### Syntax
*MultiSerialInstance*.available(**)

### Parameters
*none

### Returns
number of bytes available to read

### Example
    #include <MultiSerial.h>
    
    // Create a MultiSerial instance to communicate with port 0 on shield 0x4D
    MultiSerial msInstance = MultiSerial(0x4D, 0);
    
    void setup() {
      // Set up port for communications at 9600 bits per second
      msInstance.begin(9600);
    }
    
    void loop () {
      if(msInstance.available() > 0) {
        // take action only if there are bytes available for reception on the port
      }
    }
## store()

### Description
Store one byte of data associated with a specific shield and port.  This byte is stored on the shield itself, not in the Arduino's memory.  Each successive call to store() will overwrite any previous byte stored.

### Syntax
*MultiSerialInstance*.store(*data*)

### Parameters
*one* byte of data to associate with a specific port on a specific shield

### Returns
nothing

### Example
    #include <MultiSerial.h>
    
    // Create a MultiSerial instance to communicate with port 0 on shield 0x4D
    MultiSerial msInstance = MultiSerial(0x4D, 0);
    
    void setup() {
      // Store the number 42 associated with shield 4D, port 0 (as initialized above)
      msInstance.write(42);
    }
## retrieve()

### Description
Retrieve the byte of data aassociated with a specific shield and port, that had been stored earlier.

### Syntax
*MultiSerialInstance*.retrieve(**)

### Parameters
*none

### Returns
one byte of data that is associated with a specific port on a specific shield

### Example
    #include <MultiSerial.h>
    
    // Create a MultiSerial instance to communicate with port 0 on shield 0x4D
    MultiSerial msInstance = MultiSerial(0x4D, 0);
    
    void setup() {
      byte data;
    
      // Store the number 42 associated with shield 4D, port 0 (as initialized above)
      msInstance.write(42);
      
      // Read back the byte associated with this shield
      data = msInstance.retrieve();
    }
## flush()

### Description
Configured the specified GPIO pin to be an input or an output.

### Syntax
*MultiSerialInstance*.flush(**)
*MultiSerialInstance*.flush(**)
*MultiSerialInstance*.flush(*pin*, *mode*);

### Parameters
*none
*none
*pin* - GPIO pin number to change
*mode* - either INPUT or OUTPUT

### Returns
nothing

### Example
    #include <MultiSerial.h>
    
    // Create a MultiSerial instance to communicate with port 0 on shield 0x4D
    MultiSerial msInstance = MultiSerial(0x4D, 0);
    
    void setup() {
      // Set up port for communications at 9600 bits per second
      msInstance.begin(9600);
      
      // the device we're talking to generates garbage as it starts, use flush() to get rid of it
      // after waiting one second for the device to start
      delay(1000);
      msInstance.flush();
    }
## digitalWrite()

### Description
Set a specified GPIO pin to a HIGH or LOW state.  Unlike the Arduino digital pins, these do not have pullup resistors, so digitalWrite when the pin is set to an input has no effect.

### Syntax
*MultiSerialInstance*.digitalWrite(*pin*, *value*)

### Parameters
*pin* - GPIO pin number to change
*value* - HIGH or LOW

### Returns
nothing

### Example
    #include <MultiSerial.h>
    
    // Create a MultiSerial instance to communicate with port 0 on shield 0x4D
    MultiSerial msInstance = MultiSerial(0x4D, 0);
    
    void setup() {
      // Configure GPIO pin 0 as an output
      msInstance.pinMode(0, OUTPUT);
      
      // Set GPIO pin 0 to a HIGH level
      msInstance.digitalWrite(0, HIGH);
    }
## digitalRead()

### Description
Read the current value of a specific GPIO pin.

### Syntax
*MultiSerialInstance*.digitalRead(*pin*)

### Parameters
*pin* - GPIO pin number to read the value of

### Returns
HIGH or LOW

### Example
    #include <MultiSerial.h>
    
    // Create a MultiSerial instance to communicate with port 0 on shield 0x4D
    MultiSerial msInstance = MultiSerial(0x4D, 0);
    
    void setup() {
      byte level;
      
      // Configure GPIO pin 1 as an input
      msInstance.pinMode(1, INPUT);
    
      // Read the current value of pin 1 into the level variable
      level = msInstance.digitalRead(1);
    }
## setWordSize()

### Description
Set the data word size to use on a specific serial port.  Defaults to 8 bits when begin() is called, but can be changed if required.

### Syntax
*MultiSerialInstance*.setWordSize(*size*)

### Parameters
*size* - word size to use on the port, valid values are 5, 6, 7, or 8

### Returns
nothing

### Example
    #include <MultiSerial.h>
    
    // Create a MultiSerial instance to communicate with port 0 on shield 0x4D
    MultiSerial msInstance = MultiSerial(0x4D, 0);
    
    void setup() {
      // Set up port for communications at 9600 bits per second
      msInstance.begin(9600);
      
      // Change the word size to 7 bits
      msInstance.setWordSize(7);
    }
## enableInterrupt()

### Description
Enable specific interrupt types.

### Syntax
*MultiSerialInstance*.enableInterrupt(*types*)

### Parameters
*types* - bitmask of INT_RX and/or INT_TX for which interrupt(s) to enable

### Returns
nothing

### Example
    #include <MultiSerial.h>
    
    // Create a MultiSerial instance to communicate with port 0 on shield 0x4D
    MultiSerial msInstance = MultiSerial(0x4D, 0);
    
    void dataReceived() {
      // this will be called each time there's new data in the shield's receive buffer
    }
    
    void setup() {
      // Set up port for communications at 9600 bits per second
      msInstance.begin(9600);
      
      // Turn on the shield's receive interrupt
      msInstance.enableInterrupt(INT_RX);
      
      // Attach the Arduino's interrupt pin 0 to the 'dataReceived' function
      attachInterrupt(0, dataReceived, FALLING);
    }

## disableInterrupt()

### Description
Disable specific interrupt types.

### Syntax
*MultiSerialInstance*.disableInterrupt(*types*)

### Parameters
*types* - bitmask of INT_RX and/or INT_TX for which interrupt(s) to disable

### Returns
nothing

## print()

### Description
Outputs data to the serial port, converting numbers to their human-readable ASCII representations.

### Syntax
*MultiSerialInstance*.print(*value*)
*MultiSerialInstance*.print(*value*, *base*)
*MultiSerialInstance*.print(*string*)

### Parameters
*value* - Variable or literal to output
*base* - Base to display number in (OCT, DEC, BIN, HEX) or number of decimal places for floating point values, DEC assumed if not supplied
*string* - String to output

### Returns
nothing

### Example
    #include <MultiSerial.h>
    
    // Create a MultiSerial instance to communicate with port 0 on shield 0x4D
    MultiSerial msInstance = MultiSerial(0x4D, 0);
    
    void setup() {
      // Set up port for communications at 9600 bits per second
      msInstance.begin(9600);
      
      // Outputs "Hello, World!"
      msInstance.print("Hello, World!");
      
      // Outputs "30" in text
      msInstance.print(30);
      
      // Outputs "1E" in text
      msInstance.print(30, HEX);
    
      // Outputs "36" in text
      msInstance.print(30, OCT);
      
      // Outputs "3.14" in text
      msInstance.print(3.14159, 2);
    }


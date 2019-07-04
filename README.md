
# MotionUnit

Accelerometer/ Gyroscope library
============================

The motion unit library is a library that can be used to controll Gyroscopes, Accelerometers and the combination of both.
It has multiple abstract classes that can be extended to add more sensors. Such as the gyroscope class, accelerometer class, 
the gyrometer class(for sensors that have both gyroscopes and accelerometers) and a VR gyrometer class. 
The last class can be used to create a virtual gyrometer out of a accelerometer and a gyroscope. 
The library is structured this way that you can connect either I2C chips or SPI chips. 

At the time of writing there are 2 sensors implemented, the mpu60x0 series and the l3g4200d. 

Dependencies
-----
- For abstraction of communication with microcontrollers, this library uses [hwlib](http://github.com/wovo/hwlib) for it's hardware specific pins and wait functions. 

Installation
-----
- Download the library `cd ~; git clone http://github.com/daanwesterhof/MotionUnit`
- Make sure that [hwlib](http://github.com/wovo/hwlib) is installed in the home directory.
  
 
Preview Code
----  

`  
auto sda = hwlib::target::pin_oc(hwlib::target::pins::sda1);  
auto scl = hwlib::target::pin_oc(hwlib::target::pins::scl1);  
auto i2c_bus = hwlib::i2c_bus_bit_banged_sda_scl(sda, scl);  
auto mpu = mpu60x0(i2c_bus, 0x68);  
  
// used to get the chip out of sleep mode when powered on  
mpu.reset_chip();  
  
while(true){  
  // print the results, this function will return the angle of the x-axis.  
  hwlib::cout << int(mpu.get_x_ACC()) << "\n";  
}  
`  



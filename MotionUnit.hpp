///////////////////////////////////////////////////////////////
//          Copyright Daan Westerhof 2019.                   //
// Distributed under the Boost Software License, Version 1.0.//
//    (See accompanying file LICENSE_1_0.txt or copy at      //
//          https://www.boost.org/LICENSE_1_0.txt)           //
///////////////////////////////////////////////////////////////

#ifndef MOTIONUNIT_LIB
#define MOTIONUNIT_LIB


#include "hwlib.hpp"

#include "spi/spi_base.hpp"
#include "spi/spi_bitbang.hpp"

#include <cmath>

namespace MUNIT{

    /**
     * @class MotionUnit
     * @author Daan Westerhof
     * @date 01/07/19
     * @file MotionUnit.hpp
     * @brief  MotionUnit
     * This is the super class which holds the write and read transaction for i2c and spi.
     * With the given paramterers of the constructor it decides wich protocol to use.
    */



class MotionUnit{
private:
    hwlib::i2c_bus_bit_banged_scl_sda i2c_dummy = hwlib::i2c_bus_bit_banged_scl_sda(hwlib::pin_oc_dummy, hwlib::pin_oc_dummy);
    hwlib_ex::spi_bitbang spi_dummy = hwlib_ex::spi_bitbang(hwlib::pin_out_dummy, hwlib::pin_out_dummy, hwlib::pin_in_dummy, {0,0,0});
    hwlib::i2c_primitives & i2c_bus;
    hwlib_ex::spi_base_bus & spi_bus;
    hwlib::pin_out & csn;
    bool spi_used;
    uint8_t CHIP_ID;
public:
        /**
         * @brief empty MotionUnit constructor
         * 
         * initializes the MotionUnit with only dummy values
         */
    MotionUnit() : i2c_bus(i2c_dummy), spi_bus(spi_dummy), csn(hwlib::pin_out_dummy), CHIP_ID(0) {}
    
        /**
         * @brief i2c MotionUnit constructor
         * 
         * initializes the class for i2c communication, the spi buss will be initialized with a dummy.
         * @param i2c_bus the i2c bus that is connected with the chip
         * @param CHIP_ID the adress of the chip
         */
    MotionUnit(hwlib::i2c_primitives & i2c_bus, uint8_t CHIP_ID) : i2c_bus(i2c_bus), spi_bus(spi_dummy), csn(hwlib::pin_out_dummy), spi_used(false), CHIP_ID(CHIP_ID) {}
    
        /**
         * @brief spi MotionUnit constructor
         * 
         * initializes the class for spi communication, the i2c bus will be intialized with a dummy.
         * @param spi_bus the i2c bus that is connected to the chip
         * @param csn the chip select pin for the chip
         */
    MotionUnit(hwlib_ex::spi_base_bus & spi_bus, hwlib::pin_out & csn) : i2c_bus(i2c_dummy), spi_bus(spi_bus), csn(csn), spi_used(true), CHIP_ID(0) {}
    
        /**
         * @brief write register
         * 
         * This function writes a byte to a adress of the chip
         * @param adress the adress of the register where you want to write data to 
         * @param data the data you want to write
         */
    
    
    virtual void write_register(uint8_t adress, uint8_t data){
        if(spi_used){
            spi_bus.transaction(csn).write_byte(adress).write_byte(data);
        }else{
            hwlib::i2c_write_transaction transaction(i2c_bus, CHIP_ID);
            transaction.write(adress);
            transaction.write(data);
        }
    }
     
        /**
         * @brief read register
         * 
         * This function reads the byte stored at the given adres
         * @param adress the adress of the register where you wish to read the data from
         * @return the byte read from the register
         */
     
    virtual uint8_t read_register(uint8_t adress){
        if(spi_used){
            return spi_bus.transaction(csn).write_byte(adress).read_byte();
        }else{
            uint8_t data;
            hwlib::i2c_write_transaction(i2c_bus, CHIP_ID).write(adress);
            hwlib::i2c_read_transaction transaction(i2c_bus, CHIP_ID);
            data = transaction.read_byte();
            return data;
        }
    }
      
        /**
         * @brief read n registers
         * 
         * Set the values from the given array to the values of the data read
         * @param adress adres of the register where you wish to start the reading from
         * @param data the array where the data should be placed
         * @param amount the amount of bytes to be read
         */
  
    virtual void read_n_registers(uint8_t adress, uint8_t data[], int amount){
        if(spi_used){
            spi_bus.transaction(csn).write_byte(adress).write_read(amount, nullptr, data);
        }else{
            hwlib::i2c_write_transaction(i2c_bus, CHIP_ID).write(adress);
            hwlib::i2c_read_transaction(i2c_bus, CHIP_ID).read(data, amount);
        }
    }  

    
        /**
         * @brief get the x angle
         * 
         * get the angle of the x axis.
         * @return returns the angle -90->90 degrees
         */
    virtual double get_x_angle() {return 0;}
    
        /**
         * @brief get the y angle
         * 
         * get the angle of the y axis 
         * @return returns the angle -90->90 degrees
         */
    virtual double get_y_angle() {return 0;}
    
         /**
         * @brief get the Z angle
         * 
         * get the angle of the Z axis 
         * @return returns the angle -90->90 degrees
         */
    virtual double get_z_angle() {return 0;} 
    
    
    /**
     * @brief get temp
     * 
     * fetches the temperature of the sensors internal termometer
     * @return returns the temperature
     */
    virtual int get_temp() {return 0;}
    
    virtual uint8_t get_id() {return 0;}
    virtual void reset_chip() {}
  
}; //end of class MotionUnit

    /**
     * @class Gyroscope
     * @author Daan
     * @date 01/07/19
     * @brief Gyroscope
     * This sub class is an abstract Gyroscope class wich holds the basic functions required for a gyroscope.
     * It its a virtual inherit from the Motion Unit class to avoid ambiguity.
     */

class Gyroscope : virtual public MotionUnit{
protected:
    double current_angle_x;
    double current_angle_y;
    double current_angle_z;
public:
    
        /**
         * @brief empty Gyroscope constructor
         * 
         * calls the empty MotionUnit constructor
         */
    Gyroscope() : MotionUnit() {}
    
        /**
         * @brief i2c Gyroscope constructor
         * 
         * calls the i2c MotionUnit constructor
         * @param i2c_bus the i2c bus that is connected to the gyroscope
         * @param CHIP_ID the adres of the gyroscope
         */
    Gyroscope(hwlib::i2c_primitives & i2c_bus, uint8_t CHIP_ID) : MotionUnit(i2c_bus, CHIP_ID) {}
    
        /**
         * @brief spi Gyroscope constructor
         * 
         * calls the spi MotionUnit constructor
         * @param spi_bus the spi bus connected to the gyroscope
         * @param csn the chip select pin connected to the gyrsocope
         */
    Gyroscope(hwlib_ex::spi_base_bus & spi_bus, hwlib::pin_out & csn) : MotionUnit(spi_bus, csn) {}     

            /**
             * @brief get x ar
             * 
             * gets the angular rate of the x axis
             * @return returns the angular rate
             */
        virtual int get_x_ar() {return 0;}
        
        
            /**
             * @brief get y ar
             * 
             * gets the angular rate of the y axis
             * @return returns the angular rate
             */
        virtual int get_y_ar() {return 0;}
        
        
            /**
             * @brief get z ar
             * 
             * gets the angular rate of the z axis
             * @return returns the angular rate
             */
        virtual int get_z_ar() {return 0;}
}; // End of class Gyroscope

    /**
     * @class Accelerometer
     * @author Daan
     * @date 01/07/19
     * @brief Accelerometer
     * This sub class is an abstract Accelerometer class wich holds the basic functions required for a Accelerometer.
     * It its a virtual inherit from the MotionUnit class to avoid ambiguity.
     */

class Accelerometer : virtual public MotionUnit{
    
public:
        /**
         * @brief empty Accelerometer constructor
         * 
         * calls the empty MotionUnit constructor
         */
    Accelerometer() : MotionUnit() {}
    
        /**
         * @brief i2c Accelerometer constructor
         * 
         * calls the i2c MotionUnit constructor
         * @param i2c_bus the i2c bus connected to the accelerometer
         * @param CHIP_ID the i2c adress of the accelerometer
         */
    Accelerometer(hwlib::i2c_primitives & i2c_bus, uint8_t CHIP_ID) : MotionUnit(i2c_bus, CHIP_ID) {}
    
        /**
         * @brief spi Accelerometer constructor
         * 
         * calls the spi MotionUnit constructor
         * @param spi_bus the spi bus connected to the accelerometer
         * @param csn the chip select pin connected to the accelerometer
         */
    Accelerometer(hwlib_ex::spi_base_bus & spi_bus, hwlib::pin_out & csn) : MotionUnit(spi_bus, csn) {}
    
        /**
         * @brief get x acc
         * 
         * get the accelerometer values of the x axis
         * @return returns the accelerometer value
         */
    virtual int get_x_acc() {return 0;}
    
        /**
         * @brief get y acc
         * 
         * get the accelerometer values of the y axis
         * @return returns the accelerometer value
         */
    virtual int get_y_acc() {return 0;}
    
        /**
         * @brief get z acc
         * 
         * get the accelerometer values of the z axis
         * @return returns the accelerometer value
         */
    virtual int get_z_acc() {return 0;}
    
}; // End of class Accelerometer

    /**
     * @class Gyrometer
     * @author Daan
     * @date 01/07/19
     * @brief Gyrometer
     * This abstract class combines the Gyrosocpe and Accelerometer classes to a single class.
     * This is for chips/ sensors who have both a Gyroscope and Accelerometer, for instance a mpu6050.
     */

class Gyrometer : public Gyroscope, public Accelerometer{
public:

        /**
         * @brief empty Gyrometer constructor
         * 
         * calls the empty constructors of all super classes becous of virtual inheritance
         */
    Gyrometer() : MotionUnit(), Gyroscope(), Accelerometer() {}
    
        /**
         * @brief i2c Gyrometer constructor
         * 
         * calls the i2c constructors of all super classes becous of virtual inheritance
         * @param i2c_bus the i2c bus connected to the Gyrometer
         * @param CHIP_ID the adress of the connected Gyrometer
         */
    Gyrometer(hwlib::i2c_primitives & i2c_bus, uint8_t & CHIP_ID) : MotionUnit(i2c_bus, CHIP_ID), Gyroscope(i2c_bus, CHIP_ID), Accelerometer(i2c_bus, CHIP_ID) {}
    
        /**
         * @brief spi Gyrometer constructor
         * 
         * calls the spi constructor of all super classes becous of virtual inheritance
         * @param spi_bus the spi bus connected to the Gyrometer
         * @param csn the chip select pin connected to the Gyrometer
         */
    Gyrometer(hwlib_ex::spi_base_bus & spi_bus, hwlib::pin_out & csn) : MotionUnit(spi_bus, csn), Gyroscope(spi_bus, csn), Accelerometer(spi_bus, csn) {}
    
}; //End of class Gyrometer


    /**
     * @class VrGyrometer
     * @author Daan
     * @date 01/07/19
     * @brief VrGyrometer
     * This abstract class allows the user to combine a single Gyroscope and Accelerometer into a Virtual Gyrometer.
     * 
     */
 
class VrGyrometer : public Gyrometer{
    Accelerometer & Accel;
    Gyroscope &  Gyro;
public:

        /**
         * @brief VrGyrometer constructor
         * 
         * combines a Accelerometer and a Gyorscope into one virtual chip(virtual gyrometer)
         * It calls the Empty constructor to the Gyrometer class.
         * @param Accel the Accelerometer that is wished to be used
         * @param Gyro the Gyroscope that is wished to be used
         */
    VrGyrometer(Accelerometer & Accel, Gyroscope & Gyro) : Gyrometer(), Accel(Accel), Gyro(Gyro) {}
    
    int get_x_ar() override{
        return Gyro.get_x_ar();
    }
    int get_y_ar() override{
        return Gyro.get_y_ar();
    }
    int get_z_ar() override{
        return Gyro.get_y_ar();
    }
    
    int get_x_acc() override{
        return Accel.get_x_acc();
    }
    
    int get_y_acc() override{
        return Accel.get_y_acc();
    }
    
    int get_z_acc() override{
        return Accel.get_z_acc();
    }
    
    int get_temp() override{
        return (Accel.get_temp() + Gyro.get_temp())/2;
    }
    
    double get_x_angle() override{
        return Gyro.get_x_angle() * 0.96 + Accel.get_x_angle() * 0.04;
    }
    
    double get_y_angle() override{
        return Gyro.get_y_angle() * 0.96 + Accel.get_y_angle() * 0.04;
    }
    
    double get_z_angle() override{
        return Gyro.get_z_angle();  
    }
}; //End of class VrGyrometer

    /**
     * @class mpu60x0
     * @author Daan
     * @date 01/07/19
     * @brief mpu60x0
     * This class holds almost all functions needed to fully operate a mpu6000 or mpu6050.
     */

class mpu60x0 : public Gyrometer{
protected:
    uint8_t gyro_fsr_mode = 0;
    uint8_t accel_fsr_mode = 0;
    float gyro_fsr_values[4] = {131, 65.5, 32.8, 16.4};
    uint16_t accel_fsr_values[4] = {16384, 8192, 4096, 2048};
public:

        /**
         * @brief i2c mpu60x0 constructor
         * 
         * This constructor asks for a i2c bus and chip adress.
         * It then calls the MotionUnit and Gyrometer i2c constructors
         * @param i2c_bus the i2c bus connected to the mpu60x0
         * @param CHIP_ID the adress of the mpu60x0
         */
    mpu60x0(hwlib::i2c_primitives & i2c_bus, uint8_t CHIP_ID = 0x68) : MotionUnit(i2c_bus, CHIP_ID), Gyrometer(i2c_bus, CHIP_ID) {} 
//    uint8_t get_id() override;    
        /**
        * @brief
        * set the gyro fsr
        * 
        * this function allows you to change the gyroscope full scale range
        *it accepts the values 0, 1, 2, 3 that stand for 250 , 500, 1000, 2000 degrees a second respectively
        */
    void set_gyro_fsr(unsigned int data);
        
        /**
        * @brief
        *set the accel fsr
        *
        *this function allows you to change the accelerometer full scale range
        *it accepts the values 0, 1, 2, 3 that stand for 2-, 4-, 8-, 16g respectively
        */
    void set_accel_fsr(unsigned int data);
         
        /**
        *\brief 
        *set the sample rate at wich registers are accesed
        *
        *set the sample rate devider to change the sample rate,
        *the base sample rate for the gyro is 8khz and the acclerometer is 1khz
        */
    void set_smpl_rate(uint8_t data);
        
        /**
        * @brief set multi master
        *
        *this function enables multi master mode, 
        *this must be enabled when the chip is used on an i2c bus with multiple masters acting on it
        */
    void set_multi_mst(const bool set);
        
        /**
        * @brief wait for external data
        * 
        * when this is enabled the data ready interrupt waits untill the sensor data
        * and external sensor data have been store in their respective registers. 
        * This makes sure that when the data ready interrupt comes on
        * all data has been synched
        */
    void set_wait_for_es(const bool set);
        
        /**
        * @brief
        * set the i2c master clock speed
        *
        * when the mpu60x0 acts as a master to its slaves,
        * set its clock speed devider with this function, it accepts a 4 bit unsigned value,
        * the values coresponding to the input can be found in the register mapping sheet of the mpu60x0
        */
    void set_i2c_mst_clk(uint8_t speed);
    
        /**
         * @brief get x angular rate
         * 
         * returns the 16 bit gyroscope data of the x axis
         * @return gyroscope data of x axis
         */
    int get_x_ar() override;
    
        /**
         * @brief get y angular rate
         * 
         * returns the 16 bit gyroscope data of the y axis
         * @return gyroscope data of y axis
         */
    int get_y_ar () override;
    
        /**
         * @brief get z angular rate
         * 
         * returns the 16 bit gyroscope data of the z axis
         * @return gyroscope data of z axis
         */
    int get_z_ar () override;

   
        /**
         * @brief get temp
         * 
         * gets the remperature in celcius as measured by the internal temometer of the sensor
         * @return returns the temperatur
         */
    int get_temp() override;

   
        /**
         * @brief get x accel
         * 
         * returns the 16 bit accelerometer data of the x axis
         * @return accelerometer data of x axis
         */
    int get_x_acc() override;
    
        /**
         * @brief get y accel
         * 
         * returns the 16 bit accelerometer data of the y axis
         * @return accelerometer data of y axis
         */
    int get_y_acc() override;
    
        /**
         * @brief get z accel
         * 
         * returns the 16 bit accelerometer data of the z axis
         * @return accelerometer data of z axis
         */
    int get_z_acc() override;  

        /**
         * @brief get x angle
         * 
         * this function gets the angle of the x axis from the accelerometers
         * @return returns the angle of the x axis
         */
    double get_x_angle() override;
    
        /**
         * @brief get y angle
         * 
         * this function gets the angle of the y axis from the accelerometers
         * @return returns the angle of the y axis
         */
    double get_y_angle() override;

        /**
        * @brief external sensor shadow delay
        * when externals sensor shadow delay is enabled,
        * shadowing will be delayed untill all data has been recieved
        * 
        * @param set true = enable, false = disable
        */
    void delay_es_shadow(const bool set);
        
        /**
         * @brief set the i2c acces rate
         * 
         * When enabled the slaves(each slave can be enabled seperatly with set_slave_ar_delay ) will be accesed for 1/(1 + acces_rate_devider) per sample rate
         * @param data the devider for the acces_rate
         */
    void set_slv_acces_rate_delay(const uint8_t data);
        
        
        /**
         * @brief enable slv acces rate delay
         * 
         * enable or disable acces rate delay for each slave (1, 2, 3, 4)
         * the delay is set with the set_slv_acces_rate_delay function
         * @param slave the slave to be edited
         * @param set bool: true or false (enable or disable)
         */
    void en_slv_ar_delay(const int slave, const bool set);
    
        /**
         * @brief reset gyro paths
         * 
         * resets all difital paths, analog to digital converters and filters of the gyroscope to their power up configurations
         */
    void reset_gyro_paths();
    
        /**
         * @brief reset accel paths
         * 
         * resets all difital paths, analog to digital converters and filters of the accelerometer to their power up configurations
         */
    void reset_accel_paths();
    
        /**
         * @brief reset temp paths
         * 
         * resets all difital paths, analog to digital converters and filters of the termometer to their power up configurations
         */
    void reset_temp_paths();
    
        /**
         * @brief reset fifo
         * 
         * this function resets the entire fifo buffer
         */
    void reset_fifo();
    
        /**
         * @brief reset i2c
         * 
         * this function resets the i2c master while the i2c master is disabled ( enable_i2c_mst )
         */
    void reset_i2c();
    
        
        /**
         * @brief reset sensor paths
         * 
         * this function resets all signal paths of all sensors en resets all the data output registers
         */
    void reset_sensor_paths();
        
        
        /**
         * @brief reset_chip
         * 
         * Reset all chip registers to 0x00, except for PWRM_MGMT_1 = 0x40 and WHO_AM_I = 0x68
         */
    void reset_chip() override;
    
        /**
         * @brief enable fifo
         * 
         * enables or disables the fifo registers
         * @param set true or false (enable or disable) 
         */
    void enable_fifo(const bool set);
    
        /**
        * \brief enable i2c master
        * 
        * if enabled, the sensor acts as a master to the auxilirary i2c bus
        * if disabled the auxilirary bus is driven by the main i2c bus
        * @param set true or false (enable or disable)
        */
    void enable_i2c_mst(const bool set );
        
        
        /**
         * @brief enable sleep mode
         * 
         * this function enables or disables sleep mode, 
         * in sleep mode the sensor uses les power and is fast to start back up
         * but the i2c and spi communication is still enabled
         * @param set true or false(enable or disable)
         */
    void set_sleep_mode(const bool set);
        
        /**
        * @brief 
        * 
        * When enabled the sensor will switch between sleep and wake mode,
        to take 1 sample of the enabled sensors at the rate set by set_wakeup_ctrl 
         * the sensors that are woken are the ones where standby is not enabled.
        * @param set true or false(enable or disable)
        */
    void set_cycle_mode(const bool set);
        
        
        /**
         * @brief enable temp
         * 
         * enables or disables the sensors internal temperature sensor
         * @param set true or false(enable or disable)
         */
    void enable_temp(const bool set);
        
        
        /**
         * @brief set clock
         * set the internal clock source (defaults to 8mhz oscilator
         * 
         *  - 0 = internal 8mhz oscillator
         *  - 1 = PLL with X axis gyroscope refrence
         *  - 2 = PLL with Y axis gyroscope refrence
         *  - 3 = PLL with z axis gyroscope refrence
         *  - 4 = PLL with external 32.768KHz refremce
         *  - 5 = PLL with external 16.2MHz refrence
         *  - 6 = reserved
         *  - 7 = stops the clock and keeps the timing generator in reset
         * @param set the clock source to be set
         */
    void set_clock(const unsigned int set);
    
        
        /**
         * @brief set the wakeup frequency
         * 
         * Set the wakeup frequency of the cycle mode
         *  - 0 = 1.25Hz
         *  - 1 = 5Hz
         *  - 2 = 20Hz
         *  - 3 = 40Hz
         * @param set the frequency to be set
         */
    void set_wakeup_ctrl(const unsigned int set);
        
        /**
         * @brief enable standby x acc
         * 
         * enablefor the accelerometer x axis
         * @param set true or false(enable or disable)
         */
    void enable_standby_XACC(const bool set);
    
        /**
         * @brief enable standby y acc
         * 
         * enablefor the accelerometer y axis
         * @param set true or false(enable or disable)
         */
    void enable_standby_YACC(const bool set);
    
        /**
         * @brief enable standby z acc
         * 
         * enablefor the accelerometer z axis
         * @param set true or false(enable or disable)
         */
    void enable_standby_ZACC(const bool set);
    
        /**
         * @brief enable standby x gyro
         * 
         * enablefor the gyroscope x axis
         * @param set true or false(enable or disable)
         */
    void enable_standby_XGYR(const bool set);
    
        /**
         * @brief enable standby y gyro
         * 
         * enablefor the gyroscope y axis
         * @param set true or false(enable or disable)
         */
    void enable_standby_YGYR(const bool set);
    
        /**
         * @brief enable standby z gyro
         * 
         * enablefor the gyroscope z axis
         * @param set true or false(enable or disable)
         */
    void enable_standby_ZGYR(const bool set);
        
        /**
         * @brief get fifo count
         * 
         * returns the amount of bytes stored in the fifo buffer
         * @return returns the amount of bytes stored
         */
    uint16_t get_fifo_count();
    
    
        /**
         * @brief  get fifo data
         * returns the first byte available in the fifo buffer
         * 
         * returns a 8 bit value/ the first byte available in the fifo buffer,
         * if the buffer is empty (get_fifo_count = 0) it returns the last value that was available
         * @return 
         */
    uint8_t get_fifo_data();
        
        /**
         * @brief enable fifo for temperature
         * @param set true or false(enable or disable)
         */
    void enable_fifo_TEMP(const bool set);
    
        /**
         * @brief enable fifo for accelerometor data
         * @param set true or false(enable or disable)
         */
    void enable_fifo_ACCEL(const bool set);
    
        /**
         * @brief enable fifo for the gyroscope x axis
         * @param set true or false(enable or disable)
         */
    void enable_fifo_GYRX(const bool set);
    
        /**
         * @brief enable fifo for the gyroscope y axis
         * @param set true or false(enable or disable)
         */
    void enable_fifo_GYRY(const bool set);
    
        /**
         * @brief enable fifo for the gyroscope z axis
         * @param set true or false(enable or disable)
         */
    void enable_fifo_GYRZ(const bool set);




        /**
         * @brief enable bypass mode
         * 
         * when this is enabled the i2c master is able to directly acces the chips
            connected to the mpu's auxilirary i2c bus
         * @param set true or false(enable or disable)
         */
    void enable_bypass_mode(const bool set);


        /**
         * @brief enable i2c slave
         * 
         * enable a i2c slave
         *  - 1 = slave 1
         *  - 2 = slave 2
         *  - 3 = slave 3
         *  - 4 = slave 4
         * @param slave the slave to be enabled or disabled
         * @param set true or false(enable or disable
         */
    void i2c_slv_enable(const int slave ,const bool set);
        
        /**
         * @brief i2c set slave adress
         * 
         * set the adress of the specified i2c slave
         * @param slave the slave of wich the adress should be set
         * @param data the adress that should be set
         */
    void i2c_slv_set_address(const int slave, uint8_t data);        
        
        /**
         * @brief i2c slave set read mode
         * 
         * set the specified i2c slave to read mode (read from slave)
         * @param slave the slave to be set
         */
    void i2c_slv_set_read(const int slave);
        
        /**
         * @brief i2c slave set bytes read
         * 
         * set the amount of bytes that should be read from the specified slave
         * @param slave the slave to be set
         * @param amount the amount of data to be read
         */
    void i2c_slv_set_bytes_read(const int slave, const int amount);
        
        /**
         * @brief i2c slave set write mode
         * 
         * set the specified i2c slave to write mode(write to slave)
         * @param slave the slave to be set
         */
    void i2c_slv_set_write(const int slave);
        
        /**
         * @brief i2c slave set register
         * 
         * set the register to be written to or read from of the specified slave
         * @param slave slave to be set
         * @param adress the register adress to be set
         */
    void i2c_slv_set_reg(const int slave, const uint8_t adress);
        
    
        /**
         * @brief i2c slave do
         * 
         * set the data dat should be written to the specified slave
         * @param slave the slave where the data should be writen to
         * @param data the data to be writen to the slave
         */
    void i2c_slv_do(const int slave, const uint8_t data);
        
        /**
         * @brief enable fifo slave
         * 
         * enable or disable fifo storage for the specified slave
         * @param slave the slave which should have fifo enabled or disabled
         * @param set true or false(enable or disable)
         */
    void enable_fifo_slv(const int slave,const bool set);
        
        /**
         * @brief i2c slave swap high low
         * 
         * swap the most significant byte and the least signifivant byte of the grouped data from a slave
         * use i2c_slv_group_regs to set the way the read data is paired
         * @param slave the slave to be set
         * @param set true or false(enable or disable)
         */
    void i2c_slv_swap_hl(const int slave, const bool set);
        
        /**
         * @brief i2c slave disable register
         * 
         * disable the writing of a register adres before initiating a read or write transaction
         * @param slave the slave to be set
         * @param set true or false(disable or enable)
         */
    void i2c_slv_disable_reg(const int slave, const bool set);
        
        
        /**
         * @brief i2c slave group registers
         * 
         * set the way bytes that are read from register adresses are grouped 
         * (even then odd registers(0+1, 2+3) or odd then even registers(1+2, 3+4))
         * @param slave the slave to be set
         * @param set true or false (false = even then odd, true = odd then even)
         */

    void i2c_slv_group_regs(const int slave, const bool set);
        
        /**
         * @brief read external data
         * 
         * put the data from the extrernal data registers in a array of 24 bytes long
         * @param data the array where the data should be put
         */
    void read_ext_data(uint8_t data[]);
//        
//    
        /**
         * @brief i2c slave 4 flush
         * 
         * write the data set by i2c_slv_do to slave 4 
         */
    void i2c_slv4_flush();
        
        /**
         * @brief i2c slave 4 enable interrupt
         * 
         * enables or disables a intrupt outputted if slave 4 initiates an action
         * @param set true or false(enable or disable)
         */
    void i2c_slv4_enable_int(const bool set);
        
        /**
         * @brief i2c slave 4 get data
         * 
         * get the data read from the slave 4 register specified by i2c_slv_set_reg 
         * @return returns the data read of slave 4
         */
    uint8_t i2c_slv4_get_data();
}; //End of class mpu60x0



    /**
     * @class l3g4000d
     * @author Daan
     * @date 01/07/19
     * @brief l3g4000d
     * This class holds the basic functions needed to operate a l3g4000d gyroscope.
     */
class l3g4200d : public Gyroscope{
public:

        /**
         * @brief i2c l3g4200d constructor
         * 
         * This constructor asks for a i2c bus and chip adress.
         * It then calls the MotionUnit and Gyroscope i2c constructors
         * @param i2c_bus the i2c bus connected to the l3g4200d
         * @param CHIP_ID the adress of the l3g4200d
         */
    l3g4200d(hwlib::i2c_primitives & i2c_bus, uint8_t CHIP_ID = 0x69) : MotionUnit(i2c_bus, CHIP_ID), Gyroscope(i2c_bus, CHIP_ID){}
        
        /**
         * @brief enable x axis
         * 
         * enables the x axis of the gyroscope
         */
    void enable_x();
    
        /**
         * @brief enable y axis
         * 
         * enables the y axis of the gyroscope
         */
    void enable_y();
    
        /**
         * @brief enable z axis
         * 
         * enables the z axis of the gyroscope
         */
    void enable_z();
    
    
        /**
         * @brief disable x axis
         * 
         * disables the x axis of the gyroscope
         */
    void disable_x();
    
        /**
         * @brief disable y axis
         * 
         * disables the y axis of the gyroscope
         */
    void disable_y();
    
        /**
         * @brief disable z axis
         * 
         * disables the z axis of the gyroscope
         */
    void disable_z();
    
        /**
         * @brief get x angular rate
         * 
         * returns the angular rate of the x axis
         * @return returns the angular rate
         */
    int get_x_ar() override;
    
        /**
         * @brief get y angular rate
         * 
         * returns the angular rate of the y axis
         * @return returns the angular rate
         */
    int get_y_ar() override;
    
        /**
         * @brief get z angular rate
         * 
         * returns the angular rate of the z axis
         * @return returns the angular rate
         */
    int get_z_ar() override;
    
        /**
         * @brief check x
         * 
         * check if the x axis has new data available
         * @return bool true or false
         */
    bool check_x();
    
        /**
         * @brief check y
         * 
         * check if the y axis has new data available
         * @return bool true or false
         */
    bool check_y();
    
        /**
         * @brief check z
         * 
         * check if the z axis has new data available
         * @return bool true or false
         */
    bool check_z();


        /**
         * @brief set output data rate
         * 
         * set the output data rate of the sensor
         *  - 0 = 100Hz
         *  - 1 = 200Hz
         *  - 2 = 400Hz
         *  - 3 = 800Hz
         * @param rate the rate that should be set
         */
    void set_rate(uint8_t rate);
    
    
        /**
         * @brief enable block data update
         * 
         * when enabled it makes sure no data is updated untill al registers are read
         * this makes sure that all registers to be read are form the same time
         */
    void enable_bdu();
    
        /**
         * @brief disable block data update
         * 
         * when disable only the high and low bytes of a axis are sure to be of a same time
         */
    void disable_bdu();
        
        /**
         * @brief set sleep mode
         * 
         * enables sleep mode, the sensor uses les power and is fast awake. But not all functions are accesable
         */
    void set_sleep_mode();
        
        /**
         * @brief set power down mode
         * 
         * the chip uses very litle power, but takes a while to start back up and not all functions are accesable
         */
    void set_pwr_down_mode();
        
        /**
         * @brief set normal mode
         * 
         * The chip is fully functional and awake
         */
    void set_normal_mode();
    
        /**
         * @brief set big endean
         * sets the sensor to send the most significant byte of each data set first
         */
    void set_big_endean();
    
        /**
         * @brief set litle endean
         * sets the sensor to send the least significant byte of each data set first
         */
    void set_litle_endean();
    
        /**
         * @brief get temp
         * 
         * gets a value that represents a un specified value -1 per each celcius.
         * This can be used to calculate temperature difrences. It does not output a actual temperature.
         * @return returns the value of the register.
         */
    int get_temp() override;
    
};

}
#endif //MOTIONUNIT_LIB
#include "MotionUnit.hpp"
#include "mpu60x0_defines.hpp"

namespace MUNIT{

//#######################################################

void mpu60x0::set_gyro_fsr(unsigned int data){
    if (data < 4){
        gyro_fsr_mode = data;
        data = data << 3;
        uint8_t read = read_register(GYRO_CONFIG);
        read = read & 0b11100111;
        data = data | read;
        write_register(GYRO_CONFIG, data);
    }
}


void mpu60x0::set_accel_fsr(unsigned int data){
    if (data < 4){
        accel_fsr_mode = data;
        data = data << 3;
        uint8_t read = read_register(ACCEL_CONFIG);
        read = read & 0b11100111;
        data = data | read;
        write_register(ACCEL_CONFIG, data);
    }
}


//#######################################################

void mpu60x0::enable_fifo_TEMP(const bool set){
    uint8_t read = read_register(FIFO_EN);
    if(set){
        read = read | 0b10000000;
    }else{
        read = read & 0b01111111;
    }
    mpu60x0::write_register(FIFO_EN, read);
}

void mpu60x0::enable_fifo_ACCEL(const bool set){
    uint8_t read = read_register(FIFO_EN);
    if(set){
        read = read | 0b00001000;
    }else{
        read = read & 0b11110111;
    }
    mpu60x0::write_register(FIFO_EN, read);
}


void mpu60x0::enable_fifo_GYRX(const bool set){
    uint8_t read = read_register(FIFO_EN);
    if(set){
        read = read | 0b01000000;
    }else{
        read = read & 0b10111111;
    }
    mpu60x0::write_register(FIFO_EN, read);
}

void mpu60x0::enable_fifo_GYRY(const bool set){
    uint8_t read = read_register(FIFO_EN);
    if(set){
        read = read | 0b00100000;
    }else{
        read = read & 0b11011111;
    }
    mpu60x0::write_register(FIFO_EN, read);
}

void mpu60x0::enable_fifo_GYRZ(const bool set){
    uint8_t read = read_register(FIFO_EN);
    if(set){
        read = read | 0b00010000;
    }else{
        read = read & 0b11101111;
    }
    mpu60x0::write_register(FIFO_EN, read);
}




//####################################################

void mpu60x0::set_smpl_rate(const uint8_t data){
    write_register(SMPLRT_DIV, data);
}


void mpu60x0::set_multi_mst(const bool set){
    uint8_t read = read_register(I2C_MST_CTRL);
    if(set){
        read = read | 0b10000000;
    }else{
        read = read & 0b01111111;
    }
    write_register(I2C_MST_CTRL, read);
}

void mpu60x0::set_wait_for_es(const bool set){
    uint8_t read = read_register(I2C_MST_CTRL);
    if(set){
        read = read | 0b01000000;
    }else{
        read = read & 0b10111111;
    }
    write_register(I2C_MST_CTRL, read);
}

void mpu60x0::set_i2c_mst_clk(uint8_t speed){
    uint8_t read = read_register(I2C_MST_CTRL);
    if(speed < 16){
        read = read & 0b11110000;
//        speed = speed | 0b11110000;
        speed = speed | read;
        write_register(I2C_MST_CTRL, speed);
    }
}


//accels ######################################################################

int mpu60x0::get_x_acc(){
    int AccX = ((read_register(ACCEL_XOUT_H) << 8) | read_register(ACCEL_XOUT_L));
    if(AccX > 32768){
        AccX = AccX - 65536;
    }
    return AccX;
}

int mpu60x0::get_y_acc(){    
    int AccY = ((read_register(ACCEL_YOUT_H) << 8) | read_register(ACCEL_YOUT_L));
    if(AccY > 32768){
        AccY = AccY - 65536;
    }
    return AccY;
}

int mpu60x0::get_z_acc(){
    int AccZ = ((read_register(ACCEL_ZOUT_H) << 8) | read_register(ACCEL_ZOUT_L));
    if(AccZ > 32768){
        AccZ = AccZ - 65536;
    }
    return AccZ;
}

//gyro's ###########################################################################

int mpu60x0::get_x_ar(){
    int GyroX = ((read_register(GYRO_XOUT_H) << 8) | read_register(GYRO_XOUT_L));
    if(GyroX > 32768){
        GyroX -= 65536;
    }
    return GyroX;
}

int mpu60x0::get_y_ar(){
    int GyroY = ((read_register(GYRO_XOUT_H) << 8) | read_register(GYRO_XOUT_L));
    if(GyroY > 32768){
        GyroY -= 65536;
    }
    return GyroY;
}
int mpu60x0::get_z_ar(){
    int GyroZ = ((read_register(GYRO_XOUT_H) << 8) | read_register(GYRO_XOUT_L));
    if(GyroZ > 32768){
        GyroZ -= 65536;
    }
    return GyroZ;
}

// graden #############################################################################

double mpu60x0::get_x_angle(){
    double AccX = get_x_acc();
    double AccY = get_y_acc();
    double AccZ = get_z_acc();
    AccX = AccX / accel_fsr_values[accel_fsr_mode];
    AccY = AccY / accel_fsr_values[accel_fsr_mode];
    AccZ = AccZ / accel_fsr_values[accel_fsr_mode];
        
    double accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / 3.14) - 0.58;
    return accAngleX;
}

double mpu60x0::get_y_angle(){
    double AccX = get_x_acc();
    double AccY = get_y_acc();
    double AccZ = get_z_acc();
    AccX = AccX / accel_fsr_values[accel_fsr_mode];
    AccY = AccY / accel_fsr_values[accel_fsr_mode];
    AccZ = AccZ / accel_fsr_values[accel_fsr_mode];
        
    double accAngleY = (atan( -1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / 3.14) + 1.58;
    return accAngleY;
}

//temp ################################################################################

int mpu60x0::get_temp(){
    int data = ((read_register(TEMP_OUT_H) << 8) | read_register(TEMP_OUT_L)) / 340 + 36.53;
    return data;
}

// resets###############################################################################

void mpu60x0::reset_gyro_paths(){
    write_register(SIGNAL_PATH_RESET, 0b00000100);
}

void mpu60x0::reset_accel_paths(){
    write_register(SIGNAL_PATH_RESET, 0b00000010);
}

void mpu60x0::reset_temp_paths(){
    write_register(SIGNAL_PATH_RESET, 0b00000001);
}



// enables ###############################################################

void mpu60x0::enable_fifo(const bool set){
    uint8_t read = read_register(USER_CTRL);
    if(set){
        read = read | 0b01000000;
    }else{
        read = read & 0b10111111;
    }
    write_register(USER_CTRL, read);
}


void mpu60x0::enable_i2c_mst(const bool set){
    uint8_t read = read_register(USER_CTRL);
    if(set){
        read = read | 0b00100000;
    }else{
        read = read & 0b11011111;
    }
    write_register(USER_CTRL, read);
}


void mpu60x0::enable_bypass_mode(const bool set){
    enable_i2c_mst(false);
    uint8_t read = read_register(INT_PIN_CFG);
    if(set){
        read = read | 0b00000010;
    }else{
        read = read & 0b11111101;
    }
    write_register(INT_PIN_CFG, read);
}


//disables and resets the fifo buffer
void mpu60x0::reset_fifo(){
    enable_fifo(false);
    uint8_t read = read_register(USER_CTRL);
    read = read | 0b00000100;
    write_register(USER_CTRL, read);
}

void mpu60x0::reset_i2c(){
    uint8_t read = read_register(USER_CTRL);
    read = read | 0b00000010;
    write_register(USER_CTRL, read);
}


//this resets all paths and all store 
//values of all the sensors in the chip
void mpu60x0::reset_sensor_paths(){
    uint8_t read = read_register(USER_CTRL);
    read = read | 0b00000001;
    write_register(USER_CTRL, read);
}



void mpu60x0::reset_chip(){
    write_register(PWR_MGMT_1, 0b00000000);
}


void mpu60x0::set_sleep_mode(const bool set){
    uint8_t read = read_register(PWR_MGMT_1);
    if(set){
        read = read | 0b01000000;
    }
    else{
        read = read * 0b10111111;
    }
    write_register(PWR_MGMT_1, read);
}


void mpu60x0::set_cycle_mode(const bool set){
    uint8_t read;
    if(set){
        enable_temp(false);
        read = read_register(PWR_MGMT_1);
        read = read & 0b10111111;
        read = read | 0b00100000;
    }else{
        enable_temp(true);
        read = read_register(PWR_MGMT_1);
        read = read & 0b10011111;
    }
    write_register(PWR_MGMT_1, read);
}


void mpu60x0::enable_temp(const bool set){
    uint8_t read = read_register(PWR_MGMT_1);
    if(set){
        read = read | 0b00001000;
    }else{
        read = read & 0b11110111;
    }
    write_register(PWR_MGMT_1, read);
}


void mpu60x0::set_clock(const unsigned int set){
    if(set < 8){
        uint8_t read = read_register(PWR_MGMT_1);
        read = read & 0b11111000;
        read = read | set;
        write_register(PWR_MGMT_1, read);
    }
}

void mpu60x0::set_wakeup_ctrl(const unsigned int set){
    if(set < 4){
        uint8_t read = read_register(PWR_MGMT_2);
        read = read & 0b00111111;
        read = read | (set << 6);
        write_register(PWR_MGMT_2, read);
    }
}


void mpu60x0::enable_standby_XACC(const bool set){
    uint8_t read = read_register(PWR_MGMT_2);
    if(set){
        read = read | 0b00100000;
    }else{
        read = read & 0b11011111;
    }
    write_register(PWR_MGMT_2, read);
}

void mpu60x0::enable_standby_YACC(const bool set){
    uint8_t read = read_register(PWR_MGMT_2);
    if(set){
        read = read | 0b00010000;
    }else{
        read = read & 0b11101111;
    }
    write_register(PWR_MGMT_2, read);
}

void mpu60x0::enable_standby_ZACC(const bool set){
    uint8_t read = read_register(PWR_MGMT_2);
    if(set){
        read = read | 0b00001000;
    }else{
        read = read & 0b11110111;
    }
    write_register(PWR_MGMT_2, read);
}


void mpu60x0::enable_standby_XGYR(const bool set){
    uint8_t read = read_register(PWR_MGMT_2);
    if(set){
        read = read | 0b00000100;
    }else{
        read = read & 0b11111011;
    }
    write_register(PWR_MGMT_2, read);
}

void mpu60x0::enable_standby_YGYR(const bool set){
    uint8_t read = read_register(PWR_MGMT_2);
    if(set){
        read = read | 0b00000010;
    }else{
        read = read & 0b11111101;
    }
    write_register(PWR_MGMT_2, read);
}

void mpu60x0::enable_standby_ZGYR (const bool set){
    uint8_t read = read_register(PWR_MGMT_2);
    if(set){
        read = read | 0b00000001;
    }else{
        read = read & 0b11111110;
    }
    write_register(PWR_MGMT_2, read);
}


uint16_t mpu60x0::get_fifo_count(){
    return ((read_register(FIFO_COUNTH) << 8) | read_register(FIFO_COUNTL));
}


uint8_t mpu60x0::get_fifo_data(){
    return read_register(FIFO_R_W);
}


//uint8_t get_id(){
//    return read_register(WHO_AM_I);
//}



// SLAVE FUNCTIONS ################################################################################
void mpu60x0::i2c_slv_enable(const int slave, const bool set){
    if(slave == 0){
        uint8_t read = read_register(I2C_SLV0_CTRL);
        if(set){
            read = read | 0b10000000;
        }else{
            read = read & 0b01111111;
        }
        write_register(I2C_SLV0_CTRL, read);
    }else if(slave == 1){
        uint8_t read = read_register(I2C_SLV1_CTRL);
        if(set){
            read = read | 0b10000000;
        }else{
            read = read & 0b01111111;
        }
        write_register(I2C_SLV1_CTRL, read);
    }else if(slave == 2){
        uint8_t read = read_register(I2C_SLV2_CTRL);
        if(set){
            read = read | 0b10000000;
        }else{
            read = read & 0b01111111;
        }
        write_register(I2C_SLV2_CTRL, read);
    }else if(slave == 3){
        uint8_t read = read_register(I2C_SLV3_CTRL);
        if(set){
            read = read | 0b10000000;
        }else{
            read = read & 0b01111111;
        }
        write_register(I2C_SLV3_CTRL, read);
    }
}


void mpu60x0::i2c_slv_set_address(const int slave, const uint8_t data){
    if(slave == 0){
        write_register(I2C_SLV0_ADDR, data);
    }else if(slave == 1){
        write_register(I2C_SLV1_ADDR, data);
    }else if(slave == 2){
        write_register(I2C_SLV2_ADDR, data);
    }else if(slave == 3){
        write_register(I2C_SLV3_ADDR, data);
    }else if(slave == 4){
        write_register(I2C_SLV4_ADDR, data);
    }
}

void mpu60x0::i2c_slv_set_read(const int slave){
    if(slave == 0){
        uint8_t read = read_register(I2C_SLV0_ADDR);
        read = read | 0b10000000;
        write_register(I2C_SLV0_ADDR, read);
    }else if(slave == 1){
        uint8_t read = read_register(I2C_SLV1_ADDR);
        read = read | 0b10000000;
        write_register(I2C_SLV1_ADDR, read);
    }else if(slave == 2){
        uint8_t read = read_register(I2C_SLV2_ADDR);
        read = read | 0b10000000;
        write_register(I2C_SLV2_ADDR, read);
    }else if(slave == 3){
        uint8_t read = read_register(I2C_SLV3_ADDR);
        read = read | 0b10000000;
        write_register(I2C_SLV3_ADDR, read);
    }else if(slave == 4){
        uint8_t read = read_register(I2C_SLV4_ADDR);
        read = read | 0b10000000;
        write_register(I2C_SLV4_ADDR, read);
    }
}

void mpu60x0::i2c_slv_set_write(const int slave){
    if(slave == 0){
        uint8_t read = read_register(I2C_SLV0_ADDR);
        read = read & 0b01111111;
        write_register(I2C_SLV0_ADDR, read);
    }else if(slave == 1){
        uint8_t read = read_register(I2C_SLV1_ADDR);
        read = read & 0b01111111;
        write_register(I2C_SLV1_ADDR, read);
    }else if(slave == 2){
        uint8_t read = read_register(I2C_SLV2_ADDR);
        read = read & 0b01111111;
        write_register(I2C_SLV2_ADDR, read);
    }else if(slave == 3){
        uint8_t read = read_register(I2C_SLV3_ADDR);
        read = read & 0b01111111;
        write_register(I2C_SLV3_ADDR, read);
    }else if(slave == 4){
        uint8_t read = read_register(I2C_SLV4_ADDR);
        read = read & 0b01111111;
        write_register(I2C_SLV4_ADDR, read);
    }
}


void mpu60x0::i2c_slv_set_bytes_read(const int slave, const int amount){
    if(slave == 0){
        uint8_t read = read_register(I2C_SLV0_CTRL);
        read = read & 0b11110000;
        if(amount < 16){
            read = read | amount;
        }
        write_register(I2C_SLV0_CTRL, read);
    }else if(slave == 1){
        uint8_t read = read_register(I2C_SLV1_CTRL);
        read = read & 0b11110000;
        if(amount < 16){
            read = read | amount;
        }
        write_register(I2C_SLV1_CTRL, read);
    }else if(slave == 2){
        uint8_t read = read_register(I2C_SLV2_CTRL);
        read = read & 0b11110000;
        if(amount < 16){
            read = read | amount;
        }
        write_register(I2C_SLV2_CTRL, read);
    }else if(slave == 3){
        uint8_t read = read_register(I2C_SLV3_CTRL);
        read = read & 0b11110000;
        if(amount < 16){
            read = read | amount;
        }
        write_register(I2C_SLV3_CTRL, read);
    }
}



void mpu60x0::i2c_slv_set_reg(const int slave, const uint8_t adress){
    if(slave == 0){
        write_register(I2C_SLV0_REG, adress);
    }else if(slave == 1){
        write_register(I2C_SLV1_REG, adress);
    }else if(slave == 2){
        write_register(I2C_SLV2_REG, adress);
    }else if(slave == 3){
        write_register(I2C_SLV3_REG, adress);
    }else if(slave == 4){
        write_register(I2C_SLV4_REG, adress);
    }
}

void mpu60x0::i2c_slv_do(const int slave, const uint8_t data){
    if(slave == 0){
        write_register(I2C_SLV0_DO, data);
    }else if(slave == 1){
        write_register(I2C_SLV1_DO, data);
    }else if(slave == 2){
        write_register(I2C_SLV2_DO, data);
    }else if(slave == 3){
        write_register(I2C_SLV3_DO, data);
    }else if(slave == 4){
        write_register(I2C_SLV4_DO, data);
    }
}



void mpu60x0::delay_es_shadow(const bool set){
    uint8_t read = read_register(I2C_MST_DELAY_CTRL);
    if(set){
        read = read | 0b10000000;
    }else{
        read = read & 0b01111111;
    }
    write_register(I2C_MST_DELAY_CTRL, read);
}


void mpu60x0::set_slv_acces_rate_delay(const uint8_t data){
    uint8_t read = read_register(I2C_SLV4_CTRL);
    read = read & 0b11100000;
    read = read | data;
    write_register(I2C_SLV4_CTRL, read);
}



void mpu60x0::en_slv_ar_delay(const int slave, const bool set){
    if(slave == 0){
        uint8_t read = read_register(I2C_MST_DELAY_CTRL);
        if(set){
            read = read | 0b00000001;
        }else{
            read = read & 0b11111110;
        }
        write_register(I2C_MST_DELAY_CTRL, read);
    }else if(slave == 1){
        uint8_t read = read_register(I2C_MST_DELAY_CTRL);
        if(set){
            read = read | 0b00000010;
        }else{
            read = read & 0b11111101;
        }
        write_register(I2C_MST_DELAY_CTRL, read);
    }else if(slave == 2){
        uint8_t read = read_register(I2C_MST_DELAY_CTRL);
        if(set){
            read = read | 0b00000100;
        }else{
            read = read & 0b11111011;
        }
        write_register(I2C_MST_DELAY_CTRL, read);
    }else if(slave == 3){
        uint8_t read = read_register(I2C_MST_DELAY_CTRL);
        if(set){
            read = read | 0b00001000;
        }else{
            read = read & 0b11110111;
        }
        write_register(I2C_MST_DELAY_CTRL, read);
    }else if(slave == 4){
        uint8_t read = read_register(I2C_MST_DELAY_CTRL);
        if(set){
            read = read | 0b00010000;
        }else{
            read = read & 0b11101111;
        }
        write_register(I2C_MST_DELAY_CTRL, read);
    }
}


void mpu60x0::enable_fifo_slv(const int slave, const bool set){
    if(slave == 0){
        uint8_t read = read_register(FIFO_EN);
        if(set){
            read = read | 0b00000001;
        }else{
            read = read & 0b11111110;
        }
        write_register(FIFO_EN, read);
    }else if(slave == 1){
        uint8_t read = read_register(FIFO_EN);
        if(set){
            read = read | 0b00000010;
        }else{
            read = read & 0b11111101;
        }
        write_register(FIFO_EN, read);
    }else if(slave == 2){
        uint8_t read = read_register(FIFO_EN);
        if(set){
            read = read | 0b00000100;
        }else{
            read = read & 0b11111011;
        }
        write_register(FIFO_EN, read);
    }else if(slave == 3){
        uint8_t read = read_register(PWR_MGMT_1);
        if(set){
            read = read | 0b00100000;
        }else{
            read = read & 0b11011111;
        }
        write_register(FIFO_EN, read);
    }
}


void mpu60x0::i2c_slv_swap_hl(const int slave, const bool set){
    if(slave == 0){
        uint8_t read = read_register(I2C_SLV0_CTRL);
        if(set){
            read = read | 0b01000000;
        }else{
            read = read & 0b10111111;
        }
        write_register(I2C_SLV0_CTRL, read);
    }else if(slave == 1){
        uint8_t read = read_register(I2C_SLV1_CTRL);
        if(set){
            read = read | 0b01000000;
        }else{
            read = read & 0b10111111;
        }
        write_register(I2C_SLV1_CTRL, read);
    }else if(slave == 2){
        uint8_t read = read_register(I2C_SLV2_CTRL);
        if(set){
            read = read | 0b01000000;
        }else{
            read = read & 0b10111111;
        }
        write_register(I2C_SLV2_CTRL, read);
    }else if(slave == 3){
        uint8_t read = read_register(I2C_SLV3_CTRL);
        if(set){
            read = read | 0b01000000;
        }else{
            read = read & 0b10111111;
        }
        write_register(I2C_SLV3_CTRL, read);
    }
}



void mpu60x0::i2c_slv_group_regs(const int slave, const bool set){
    if(slave == 0){
        uint8_t read = read_register(I2C_SLV0_CTRL);
        if(set){
            read = read | 0b00010000;
        }else{
            read = read & 0b11101111;
        }
        write_register(I2C_SLV0_CTRL, read);
    }else if(slave == 1){
        uint8_t read = read_register(I2C_SLV1_CTRL);
        if(set){
            read = read | 0b00010000;
        }else{
            read = read & 0b11101111;
        }
        write_register(I2C_SLV1_CTRL, read);
    }else if(slave == 2){
        uint8_t read = read_register(I2C_SLV2_CTRL);
        if(set){
            read = read | 0b00010000;
        }else{
            read = read & 0b11101111;
        }
        write_register(I2C_SLV2_CTRL, read);
    }else if(slave == 3){
        uint8_t read = read_register(I2C_SLV3_CTRL);
        if(set){
            read = read | 0b00010000;
        }else{
            read = read & 0b11101111;
        }
        write_register(I2C_SLV3_CTRL, read);
    }
}


void mpu60x0::i2c_slv_disable_reg(const int slave, const bool set){
    if(slave == 0){
        uint8_t read = read_register(I2C_SLV0_CTRL);
        if(set){
            read = read | 0b00100000;
        }else{
            read = read & 0b11011111;
        }
        write_register(I2C_SLV0_CTRL, read);
    }else if(slave == 1){
        uint8_t read = read_register(I2C_SLV1_CTRL);
        if(set){
            read = read | 0b00100000;
        }else{
            read = read & 0b11011111;
        }
        write_register(I2C_SLV1_CTRL, read);
    }else if(slave == 2){
        uint8_t read = read_register(I2C_SLV2_CTRL);
        if(set){
            read = read | 0b00100000;
        }else{
            read = read & 0b11011111;
        }
        write_register(I2C_SLV2_CTRL, read);
    }else if(slave == 3){
        uint8_t read = read_register(I2C_SLV3_CTRL);
        if(set){
            read = read | 0b00100000;
        }else{
            read = read & 0b11011111;
        }
        write_register(I2C_SLV3_CTRL, read);
    }else if(slave == 4){
        uint8_t read = read_register(I2C_SLV4_CTRL);
        if(set){
            read = read | 0b00100000;
        }else{
            read = read & 0b11011111;
        }
        mpu60x0::write_register(FIFO_EN, read);
    }
}

void mpu60x0::i2c_slv4_flush(){
    uint8_t read = read_register(I2C_SLV4_CTRL);
    read = read | 0b10000000;
    write_register(I2C_SLV4_CTRL, read);
}

void mpu60x0::i2c_slv4_enable_int(const bool set){
    uint8_t read = read_register(I2C_SLV4_CTRL);
    if(set){
        read = read | 0b01000000;
    }else{
        read = read & 0b10111111;
    }
    write_register(FIFO_EN, read);
}

uint8_t mpu60x0::i2c_slv4_get_data(){
    return read_register(I2C_SLV4_DI);
}

void mpu60x0::read_ext_data(uint8_t data[]){
    read_n_registers(EXT_SENS_DATA_00, data, 24);
}


}
#include "MotionUnit.hpp"
#include "l3g4200d_defines.hpp"


//#################################################### enable or disable
    
void l3g4200d::enable_x(){
    uint8_t read = MotionUnit::read_register(CTRL_REG1);
    uint8_t bite = (read | 0b00000001);
    MotionUnit::write_register(CTRL_REG1, bite);
}

void l3g4200d::disable_x(){
    uint8_t read = MotionUnit::read_register(CTRL_REG1);
    uint8_t bite = (read & 0b11111110);
    MotionUnit::write_register(CTRL_REG1, bite);
}

void l3g4200d::enable_y(){
    uint8_t read = MotionUnit::read_register(CTRL_REG1);
    uint8_t bite = (read | 0b00000010);
    MotionUnit::write_register(CTRL_REG1, bite);
}

void l3g4200d::disable_y(){
    uint8_t read = MotionUnit::read_register(CTRL_REG1);
    uint8_t bite = (read & 0b11111101);
    MotionUnit::write_register(CTRL_REG1, bite);
}

void l3g4200d::enable_z(){
    uint8_t read = MotionUnit::read_register(CTRL_REG1);
    uint8_t bite = (read | 0b00000100);
    MotionUnit::write_register(CTRL_REG1, bite);
}

void l3g4200d::disable_z(){
    uint8_t read = MotionUnit::read_register(CTRL_REG1);
    uint8_t bite = (read & 0b11111011);
    MotionUnit::write_register(CTRL_REG1, bite);
}


void l3g4200d::enable_bdu(){
    uint8_t read = MotionUnit::read_register(CTRL_REG4);
    uint8_t data = (read | 0b10000000);
    MotionUnit::write_register(CTRL_REG4, data);
}

void l3g4200d::disable_bdu(){
    uint8_t read = MotionUnit::read_register(CTRL_REG4);
    uint8_t data = (read & 0b01111111);
    MotionUnit::write_register(CTRL_REG4, data);
}
    
//##################################################### get values
    
int l3g4200d::get_x_ar(){
    return (MotionUnit::read_register(OUT_X_H) << 8) | MotionUnit::read_register(OUT_X_L);
}

int l3g4200d::get_y_ar(){
    return (MotionUnit::read_register(OUT_Y_H) << 8) | MotionUnit::read_register(OUT_Y_L);
}

int l3g4200d::get_z_ar(){
    return (MotionUnit::read_register(OUT_Z_H) << 8) | MotionUnit::read_register(OUT_Z_L);
}


//#################################################### check data ready

bool l3g4200d::check_x(){
    uint8_t read = MotionUnit::read_register(STATUS_REG);
    if((read & 0b00000001) == 0b00000001){
        return true;
    }else return false;
    
}

bool l3g4200d::check_y(){
    uint8_t read = MotionUnit::read_register(STATUS_REG);
    if((read & 0b00000010) == 0b00000010){
        return true;
    }else return false;
    
}


bool l3g4200d::check_z(){
    uint8_t read = MotionUnit::read_register(STATUS_REG);
    if((read & 0b00000100) == 0b00000100 ){
        return true;
    }else return false;
    
}



//#################################################### sleep- pwr down- normal mode

void l3g4200d::set_sleep_mode() {
    uint8_t read = MotionUnit::read_register(CTRL_REG1);
    uint8_t bite = (read & 0b11111000);
    bite = bite  | 0b00001000;
    MotionUnit::write_register(CTRL_REG1, bite);
}


void l3g4200d::set_pwr_down_mode() {
    uint8_t read = MotionUnit::read_register(CTRL_REG1);
    uint8_t bite = (read & 0b11110111);
    MotionUnit::write_register(CTRL_REG1, bite);
}

void l3g4200d::set_normal_mode(){
    uint8_t read = MotionUnit::read_register(CTRL_REG1);
    uint8_t bite = (read | 0b00001111);
    MotionUnit::write_register(CTRL_REG1, bite);
}

//#################################################### other

void l3g4200d::set_big_endean(){
    uint8_t read = MotionUnit::read_register(CTRL_REG4);
    uint8_t data = (read | 01000000);
    MotionUnit::write_register(CTRL_REG4, data);
}
void l3g4200d::set_litle_endean(){    //deafault mode on l3g4200d
    uint8_t read = MotionUnit::read_register(CTRL_REG4);
    uint8_t data = (read & 10111111);
    MotionUnit::write_register(CTRL_REG4, data);
}
    

void l3g4200d::set_rate(uint8_t rate){
    uint8_t read = MotionUnit::read_register(CTRL_REG1);
    uint8_t data = read | rate << 6;
    MotionUnit::write_register(CTRL_REG1, data);
}


int l3g4200d::get_temp(){
    return MotionUnit::read_register(OUT_TEMP);
}
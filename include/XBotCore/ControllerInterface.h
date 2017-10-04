#ifndef __X_BOT_CONTROLLER_INTERFACE_H__
#define __X_BOT_CONTROLLER_INTERFACE_H__


class ControllerInterface  {
  
public:
  
    virtual void control_init(void) = 0;
    
    virtual int control_loop(void) = 0;   
    
};

#endif

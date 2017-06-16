#include <XBotInterface/StateMachine.h>


namespace myfsm{

    
    struct SharedData {
      
      XBot::RobotInterface::Ptr _robot;
     
    };
    
    class MacroState : public  XBot::FSM::State< MacroState , SharedData > {
      
    public:
	
	virtual void entry(const XBot::FSM::Message& msg) {};
	virtual void react(const XBot::FSM::Event& e){};
      
    };  

     _STATE_DEFINITION_
      
}

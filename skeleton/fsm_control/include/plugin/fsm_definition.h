#include <XBotInterface/StateMachine.h>


namespace myfsm{

/*Example how to define a custom Event*/
/*  class MyEvent : public XBot::FSM::Event {

    public:

      MyEvent(int id): id(id) {}
      
      int id;    

    };
*/

/*Example how to define a custom Message*/   
/*  class MyMessage : public XBot::FSM::Message {

    public:
      
      MyMessage (int id):id(id){};
      
      int id;
	
    };
*/
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

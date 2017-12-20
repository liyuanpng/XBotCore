#ifndef __REQUEST_INTERFACE_WEBSERVER_H__
#define __REQUEST_INTERFACE_WEBSERVER_H__

#include "rapidjson/document.h"
#include <Web/WebRobotState.h>

using namespace rapidjson;

class RequestInterface {
  
 
  public:
    
    virtual void SetData(void* buff) = 0;
    
    virtual void* GetData() = 0;
       
};


class RequestObject: public RequestInterface {
  
private:
    
    void *buff;
  
  public:
    
    void SetData(void* buff) {
      
        this->buff = buff;
    }
    
    void* GetData() {
     
        return this->buff;
        
    }
       
};

class GetterInterface {
  
 
  public:
    
    virtual  bool GetDoubleArray(const std::string& name,std::vector<double>& vec) = 0;
    
    virtual bool GetIntDoubleMap(const std::string& name,std::map<int, double>& map) = 0;
       
};


class BinaryRequest: public GetterInterface {
  
private:
    
    std::shared_ptr<RequestInterface> reqInterface;
    
  
  public:
    
    BinaryRequest ( std::shared_ptr<RequestInterface> reqInterface): reqInterface(reqInterface){
      
      
    };
   
    bool GetDoubleArray(const std::string& name,std::vector<double>& vec){
     
      return false;
      
    }
    
    bool GetIntDoubleMap(const std::string& name,std::map<int, double>& map){
           
       return false;
      
    }
       
};

class JsonRequest: public GetterInterface {
  
private:
    
    std::shared_ptr<RequestInterface> reqInterface;
    Document d;
  
  public:
    
    JsonRequest ( std::shared_ptr<RequestInterface> reqInterface): reqInterface(reqInterface){
      
       StringStream stream((char*)reqInterface->GetData());      
       d.ParseStream(stream);
    };
    
    Document& GetDocument(){
      
      return d;
    }
    
    bool getRobotState(WebRobotStateRX& rstate){      
      
       if( d.HasMember("joint")){          
         // assert(d["joint"].isArray());
          const Value& array = d["joint"];
          for (SizeType i = 0; i < array.Size(); i++){
              const Value& obj = array[i];
              int id = obj["id"].GetInt();
              double pos_ref = obj["pos"].GetDouble();
              double vel_ref = obj["vel"].GetDouble();
              double eff_ref = obj["eff"].GetDouble();
              double stiff_ref = obj["stiff"].GetDouble();
              double damp_ref = obj["damp"].GetDouble();
              rstate.joint_id.push_back(id);
              rstate.position_ref.push_back(pos_ref);
              rstate.vel_ref.push_back(vel_ref);
              rstate.effort_ref.push_back(eff_ref);
              rstate.stiffness.push_back(stiff_ref);
              rstate.damping.push_back(damp_ref);             
          }
          
          return true;
          
        }
        
       return false;
      
    }
    
    bool GetDoubleArray(const std::string& name,std::vector<double>& vec){
      
      if( d.HasMember(name.c_str())){        
          //assert(d["link_position"].isArray());
          const Value& array = d[name.c_str()];
          for (SizeType i = 0; i < array.Size(); i++){
              double val = array[i].GetDouble();
              vec.push_back(val);   
          }
          return true;
      }
      
      return false;
      
      /*StringBuffer buffer;
      Writer<StringBuffer> writer(buffer);
      d.Accept(writer);
      std::cout <<"stringify"<< std::string(buffer.GetString()) << std::endl;*/ 
    }
    
    bool GetIntDoubleMap(const std::string& name,std::map<int, double>& map){
      
      
       if( d.HasMember(name.c_str())){          
         // assert(d["joint"].isArray());
          const Value& array = d[name.c_str()];
          for (SizeType i = 0; i < array.Size(); i++){
              const Value& obj = array[i];
              int id = obj["id"].GetInt();
              double val = obj["val"].GetDouble();
              map[id] = val;              
          }
          
          return true;
          
        }
        
       return false;
        
    }
       
};

#endif //__REQUEST_INTERFACE_WEBSERVER_H__
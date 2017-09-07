#ifndef __REQUEST_INTERFACE_WEBSERVER_H__
#define __REQUEST_INTERFACE_WEBSERVER_H__

#include "rapidjson/document.h"
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
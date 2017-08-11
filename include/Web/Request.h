#ifndef __REQUEST_INTERFACE_WEBSERVER_H__
#define __REQUEST_INTERFACE_WEBSERVER_H__

class RequestObject {
  
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

#endif //__REQUEST_INTERFACE_WEBSERVER_H__
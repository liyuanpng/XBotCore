#!/bin/sh

if [ $# -lt 2 ]; then
    echo "Error: two input parameter expected, the RT Plugin name and at least one state"
    exit 1
fi

#create plugin's folder
mkdir -p $1
cd $1

# Local copy of XCM skeleton
cp -r $ROBOTOLOGY_ROOT/external/XBotCore/skeleton/fsm_control/* .

# find and replace 
find . -maxdepth 3 -type f -not -path '*/\.*' -exec sed -i -e "s/_MODULE_PREFIX_/$1/g" {} \;
# rename
find . -maxdepth 3 -type f -not -path '*/\.*' -not -name "CMakeLists.txt" -not -name "README.md" -not -name "FindXenomai.cmake" -not -name "MacroYCMInstallLibrary.cmake" -not -name "fsm_definition.h" -not -name "fsm_state.h" -not -name "fsm_implementation.cpp" -not -name "fsm_implementationTemplate.cpp" -exec bash -c 'dir=$(dirname $0) && file=$(basename $0) && mv $0 "$dir/$1_$file"' {} $1 \;

mv "./include/plugin" "./include/$1"


#read state definition template
package_name=$1
content_definition=$(cat include/$package_name/fsm_state.h)
content_implementation=$(cat src/fsm_implementationTemplate.cpp)
content_registration="   fsm.register_state(std::make_shared<myfsm::_STATE_PREFIX_>());"    
content_init="fsm.init("\"$2\"");"
plugin_name="$1_plugin.cpp"
 

#append the created states      
#skip first argument
shift
for var in "$@"
 do
  #generate fsm_definiton
  sed -i "/_STATE_DEFINITION_/ i\ $content_definition" include/$package_name/fsm_definition.h
  #generate fsm_implementation
  sed -i "/_STATE_DEFINITION_/ i\ $content_implementation" src/fsm_implementation.cpp
  #register states
  sed -i "/_FSM_STATE_REGISTRATION_/ i\ $content_registration" src/$plugin_name
  

  #replace with the specified state name
  sed -i -e "s/_STATE_PREFIX_/$var/g" include/$package_name/fsm_definition.h  
  sed -i -e "s/_STATE_PREFIX_/$var/g" src/fsm_implementation.cpp
  sed -i -e "s/_STATE_PREFIX_/$var/g" src/$plugin_name
 done  

#init state
sed -i -e "s/_FSM_STATE_INIT_/$content_init/g" src/$plugin_name


#delete _STATE_DEFINITION_
sed -i -e "s/_STATE_DEFINITION_//g" include/$package_name/fsm_definition.h  
sed -i -e "s/_STATE_DEFINITION_//g" src/fsm_implementation.cpp
 
#delete _STATE_REGISTRATION_ and _FSM_STATE_INIT_
sed -i -e "s/_FSM_STATE_REGISTRATION_//g" src/$plugin_name
#delete template file
rm -f "src/fsm_implementationTemplate.cpp"
rm -f "include/$package_name/fsm_state.h"

echo "XBot RT FSM Plugin skeleton is ready. Have fun!"



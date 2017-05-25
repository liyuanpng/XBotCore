XCM
===

XBot Control Modules and RT plugins.

How to generate and execute a XBot RT Plugin?
==============================================

XCM provides a script that generate for you a skeleton of a XBot RT plugin: the script will be installed in the CMAKE_INSTALL_PREFIX when installing XCM.

Once execute it 

```
mkdir RT_PLUGIN_NAME
generate_XBot_RT_Plugin.sh RT_PLUGIN_NAME
```
you will have a ready to use empty skeleton for a XBot RT plugin.

Just compile it and install it using the [advr-superbuild](https://github.com/ADVRHumanoids/advr-superbuild) (check the [advr-superbuild wiki](https://github.com/ADVRHumanoids/advr-superbuild/wiki#creating-a-new-project-in-github-and-adding-it-to-the-superbuild) to understand how).

In order to run it in your gazebo model / real robot remember to add the RT_PLUGIN_NAME in the list of the XBotRTPlugins in your YAML config file:

```
XBotRTPlugins:
  plugins: ["HomingExample", "RT_PLUGIN_NAME"]
  io_plugins: []
```

Please check this [video](https://www.youtube.com/watch?v=wJXCLhtS7T0) to better understand how to generate and execute a XBot RT plugin.


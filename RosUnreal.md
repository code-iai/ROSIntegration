# Ros Integration Plugin Documentation

## ToDo:
1. Adjust callback so it prints out whatever data type the arm passes in
    - Also need to adjust so you can subscribe to multiple topics (one for each joint im assuming)
    - Would just need to add multiple subscribe calls I believe
    - Unless they're publishing to one topic and nesting them inside a dictionary
    - issue may be that its registering in websocket connection but not in websocket override
    - Adjust MoveItMessageConverter functions to handle the custom data type
2. Fix the random unsubscription problem
3. Modularize so that you can pass host, port, topic type, topic name into a blueprint via string (make this all blueprint callable)
4.  Test out publishing to topic
5. make generate random key actually generate a random key (just hard coded right now)
6. Add a catch so your entire game doesnt crash when the rosbridge server isn't running
7. Document/adjust to accomodate dev standards

## HOW TO RUN:

### Unreal
1. Make sure connected to same wifi as rosbridge server
2. Run ipconfig on rosbridge server computer
3. Plug that value into RosIntegrationGameInstance.h ROSBridgeServerHosts, as well as RosInstance.cpp ROSBridgeServerHosts
4. Find what the topic name is, set topic in constructor of ARosActor.cpp = topic name
5. Find data type of topic `ros2 topic list -t`
    - Testing if the topic is actually subsribed to (Linux)
    `Ros2 topic info /topic_name --verbose`
6. Insert data type into ExampleTopic->Init in ARosActor.cpp

### Running Rosbridge Server
1. Source environment:
    - `source /opt/ros/humble/setup.bash`
    - `source ~/arm_publisher_test/install/setup.bash`
    - Basically just runs the setup scripts for ros2 and cyclonedds so that your environment variables are set for those and allows you to locate where they are
2. If running on a different computer than Unreal and you're running wsl, you need to forward your wsl port to windows so you can connect to it from a different computer
    - `netsh interface portproxy add v4tov4 listenport=9090 listenaddress=0.0.0.0 connectport=9090 connectaddress=172.29.251.199`
        - connect address is your wsl ip
    - `netsh advfirewall firewall add rule name="RosBridge 9090" dir=in action=allow protocol=TCP localport=9090`
3. Run `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`

 Make sure you have your publisher and bridge server environment variables ROS_DOMAIN_ID set to the same domain

### Running Publisher
1. Source your environment `source /opt/ros/humble/setup.bash` `source ~/arm_publisher_test/install/setup.bash`
2. `ros2 run go2_d1_demo arm_command_node`


___
# Analysis

### Positives
- Allows you to build the Unreal game on Windows
- Just a websocket connection with the Rosbridge server so not much of a burden on the Unreal game itself (doesn't need to actually connect to Ros2)
- Minimal lag involved, Unreal doesn't have to wait for actual robot to move first (unlike OPC UA). Both Unreal and the robot are reading them at the same time, since so mirroring within the digital world should be near instantaneous
- Can also publish to Ros2, so you can control the robot in real life from the game if you want
- Can connect to any version of Ros2 (Humble, Foxy), all you need is that version of Rosbridge

### Drawbacks
- Need to run a RosBridge server in a separate process from the Unreal instance. Can potentially look into ways to ease this by having the Unreal game trigger the server to start, but you still need to maintain it separately. RosBridge needs to be run on Linux.

___
# Changes Made to Plugin
### Upgraded Plugin from UE4 to UE5
Original issues that caused plugin to not compile in UE5 (honestly not even sure how it compiled in UE4 but i digress)

1. Forward declaration issue
    - UImpl had Impl class class defined within it, but also storing a pointer to a member instance of that class which was causing forward declaration issues. Moved Impl implementation to it's own header file and included in ROSIntegration.cpp, which fixed the issue
2. Missing library imports

### Re-Wrote Websocket Class
Original websocket class was adding an extra / within the Server IP address, which was causing a GET 404 error to be returned from connection attempt.

Didn't have access to Websocket class (was being built from binaries), so needed to write own implementation that inherited from IWebSocket

New Implementation written in WebsocketOverride.cpp (see for documentation)

### Fixed Websocket connection testing
Was just returning true for websocket connection even when it wasn't connected at all. This caused errors when you tried to subscribe to a topic because there wasn't actually a websocket connection established. Adjusted testing so that logs accurately reflected status of server connection

    LogROS: Error: Connection to rosbridge 192.168.1.225:9090 (ID 0) was interrupted.
    LogROS: Display: UROSIntegrationGameInstance::Init() - connecting to rosbridge 192.168.1.225:9090 (ID 0)...
    LogROS: Display: UImpl::BeginDestroy() 
    LogROS: Display: UROSIntegrationCore ~Impl() 
    LogROS: Display: UROSIntegrationCore spawned 
    LogROS: Display: UImpl::Init() 
    LogROS: Display: Connecting to: ws://192.168.1.225:9090
    LogROS: Display: Successfully connected to rosbridge 192.168.1.225:9090 (ID 0).
    LogTemp: Display: Callback Valid, Topic successfully subscribed
    LogROS: Error: Connection to rosbridge 192.168.1.225:9090 (ID 0) was interrupted.

### Changed data send type
Was sending data over via binary format when WS was expecting json. Caused error, fixed by just making it send json data

`Error        LogTemp                   Binary SendMessage failed, WSA error: 10053`

### Added Converter Files for Custom Data Types
Each new data type required its own converter class so that Unreal could know how to cast the incoming data type

### Steps to add new data type:

    1. Make a class that inherits from UBaseMessageConverter
        - Implement ConvertIncomingMessage and  ConvertOutgoingMessage (see MessageConverter.cpp for example)
    2. Set _MessageType = TEXT("whatever your type name is")
    3. Call AddSupportedTopic(DataTypeToParseAs, Text(Type_Name)) in UTopic initializer in Topic.cpp

    4. Need to add a .msg file to unitree_arm/msg so that the rosbridge server knows how to parse that data typem (ArmString.msg for ArmString), add data type first and then variable name
    ex: string data_name

    Type: unitree_arm::msg::dds_::ArmString_
    arm_string, is just a json you can parse




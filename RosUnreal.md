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

## Positives
    - Allows you to build the Unreal game on Windows
    - Just a websocket connection with the Rosbridge server so not much of a burden on the Unreal game itself (doesn't need to actually connect to Ros2)
    - Minimal lag involved, Unreal doesn't have to wait for actual robot to move first (unlike OPC UA). Both Unreal and the robot are reading them at the same time, since so mirroring within the digital world should be near instantaneous
    - Can also publish to Ros2, so you can control the robot in real life from the game if you want
    - Can connect to any version of Ros2 (Humble, Foxy), all you need is that version of Rosbridge

## Drawbacks
    - Need to run a RosBridge server in a separate process from the Unreal instance. Can potentially look into ways to ease this by having the Unreal game trigger the server to start, but you still need to maintain it separately. RosBridge needs to be run on Linux.

# Changes Made to Plugin
### Upgraded Plugin from UE4 to UE5
Original issues that caused plugin to not compile in UE5

1. Forward declaration issue
2. Missing library imports

### Re-Wrote Websocket Class
Original websocket class was adding an extra / within the Server IP address, which was causing a GET 404 error to be returned from RosBridge.

Didn't have access to class (was being built from binaries), so needed to write own implementation that inherited from IWebSocket

New Implementation written in WebsocketOverride.cpp

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


# Configuring Plugin

## Running test publisher
    1. Cd into directory
    2. pixi run ros2 run my_package my_node
        - my_package is project, my_node is node name







## HOW TO RUN:

## Unreal
1. Make sure connected to same wifi as rosbridge server
2. Run ipconfig on rosbridge server computer
3. Plug that value into RosIntegrationGameInstance.h ROSBridgeServerHosts, as well as RosInstance.cpp ROSBridgeServerHosts
4. Find what the topic name is, set topic in constructor of ARosActor.cpp = topic name
5. Find data type of topic `ros2 topic list -t`
    - Testing if the topic is actually subsribed to (Linux)
    `Ros2 topic info /topic_name --verbose`
6. Insert data type into ExampleTopic->Init in ARosActor.cpp

## Running Rosbridge Server
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

## Running Publisher
1. Source your environment `source /opt/ros/humble/setup.bash` `source ~/arm_publisher_test/install/setup.bash`
2. `ros2 run go2_d1_demo arm_command_node`












## Todo


## Connecting to Topic

    
    Callback handle stored here: 		
         _CallbackHandle = _ROSTopic->Subscribe(std::bind(&UTopic::Impl::MessageCallback, this, std::placeholders::_1));
    
    Callback function stored in _func, need to check what that's actually doing, however if you're not actually subscribed it doesn't really matter

    Potential issue: topic type it's trying to connect to is std_msgs/String, but the topic being published is a std_msgs/msg/String


    - How do you switch the message type?
        Checking if its a child of ubasemessageconverter class, so you need to make a class that inherits from that 
        - only question is where is that class initialized?

    - Do you need to send the message via the socket handle, or does it have another way of sending?
    
    - What is subscribe id? Do I need to set that to anything special?

    - May need to override transport layer's sendMessage, because that could just not work

    Need to know how to swap message types regardless

## Steps to fixing
1. Check if it's an issue with the message actually being sent
    - It could not be getting sent from the socket connection
        - Update: its being sent but formatted wrong
    - Getting back this result: 
        - Error        LogTemp                   Binary SendMessage failed, WSA error: 10053
    - Means that the data you're sending is in the wrong format (being sent in TCP instead of Websocket protocol I believe)
    - Check to see if it's using the right transport layer? Where is it checking ws vs tcp, because right now it's sending data in tcp format and not ws from the transport_layer SendMessage
    - So either you need to adjust which send message it's using so that it sends via WebSocket version OR you need to adjust the SendMessage implementation so that it sends via Websocket protocol
    - I believe the issue right now is that it's implemented TCP style and not Websocket Style
    - Update: the issue is that it's calling the binary send method instead of the json send method, which is causing decoding issues from the rosbridge server, which expects json
    - Likely coming from websocket connection calling the wrong method from websocket override
    - Binary Connect is being called from pulisher queue thread...what that is, I don't know. It could either be calling the wrong method from there OR there could be an issue with the binary encoder
    -Now able to send json commands to rosbridge via websocket, so I know that at least works. For some reason it hasn't successfully subscribed yet, BUT since I know I'm able to send the jsons, it is likely just an argument or type issue, so should be able to solve it relatively soon
    - IT WORKS FINALLY

2. Check if it's just a data type issue with your message
    - Publishing data type of std_msg/msg/String, but message being sent was data type std_msg/String, so made the plugin compatible with std_msg/msg/String and that solved the type error. Still didn't make plugin work though
3. Check if it's an ID issue
4. Worst case it's an issue with the transport layer's implementation of send message, in which case you would need to override it and reimplement it from scratch, because that's being read from the binaries
    - Send message is returning true when it's just not true, which is very concerning
    - Should be overriden right now, so if there's an issue it's with how it's actually implemented, not with the binary version
5. Final option could be a json/bson mismatch with the bson value set to true some places but not others, but i doubt it

Fixed receive thread so that it actually receives data from the rosbridge server now, may need to adjust frequency with which it checks but as of now it's not causing any problems

Data type: Custom type from Unitree: Unitree arm::message/arm-tree-publisher

type: moveit_msgs/msg/

unitree_arm::msg::dds_::ArmString_

Only publish to one topic so that's good, but need to adjust so that plugin can handle the Unitree type

Unitree GitHub



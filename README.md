# Aria glass streamline to OpenVINS
In this demo, we want to streamline Aria glass data to OpenVINS lively, and display the estimation result. To this end, we use a python ROS publisher wrapper, along with OpenVINS display. 

The ROS publisher need to streamline data from Aria glass with its SDK, and package the received data as ROS bag (for image & IMU). 


## Streamline Aria data

### set up the streamlineManager
```python
# 1. Create DeviceClient instance, setting the IP address if specified
device_client = aria.DeviceClient()

client_config = aria.DeviceClientConfig()
if args.device_ip:
    client_config.ip_v4_address = args.device_ip
device_client.set_client_config(client_config)

# 2. Connect to Aria: this relies on adb and assumes you have it installed on your path.
# Use set_client_config to specify it manually otherwise or connect via IP address.
device = device_client.connect()

# 3. Retrieve streaming_manager
streaming_manager = device.streaming_manager

# 4. Use custom config for streaming: use profile12 (no audio) and use ephemeral certs

streaming_config = aria.StreamingConfig()
streaming_config.profile_name = args.profile_name
# Note: by default streaming uses Wifi
if args.streaming_interface == "usb":
    streaming_config.streaming_interface = aria.StreamingInterface.Usb
streaming_config.security_options.use_ephemeral_certs = True

streaming_manager.streaming_config = streaming_config

```

### start streaming, and pass in Observer class
```python
# 5. Start streaming
streaming_manager.start_streaming()

# 6. Get streaming state
streaming_state = streaming_manager.streaming_state
print(f"Streaming state: {streaming_state}")


 # 7. Create the visualizer observer and attach the streaming client
aria_visualizer = AriaVisualizer()
aria_visualizer_streaming_client_observer = AriaVisualizerStreamingClientObserver(
    aria_visualizer
)
streaming_client.set_streaming_client_observer(
    aria_visualizer_streaming_client_observer
)
streaming_client.subscribe()
```
### Observer class work
```python
# 8. Visualize the streaming data until we close the window
aria_visualizer.render_loop()
```

So we need to use this core code to do something to received data in a loop. (Can we embed the logic directly to observer?)


### clean-up when done
```python
 # 9. Stop streaming and disconnect the device
print("Stop listening to image data")
streaming_client.unsubscribe()
streaming_manager.stop_streaming()
device_client.disconnect(device)
```

# config

1. use profile 12 for recording (turn off audio)
2. aria.sdk.StreamingSubscriptionConfig: message_queue_size


# usage
In the ROS launch file, this streamline publisher node will be launched along with OpenVINS code. 

## TODO
we need to set up parameters correctly in the OpenVINS code. 


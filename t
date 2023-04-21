<launch>
    <!-- <arg name="camera_width" default="480" /> -->
    <!-- <arg name="camera_hight" default="640" /> -->
    <node pkg="prometheus_detection" type="web_cam" name="web_cam">
        <!-- 相机id -->
        <param name="cam_id" type="int" value="0" />
        <!-- 0:web_cam, 1:mipi_cam, 2:araducam -->
        <param name="camera_type" type="int" value="0" />
    </node>

    <node pkg="prometheus_detection_circlex" type="fly_detector" name="circlex_detector" output="screen" launch-prefix="bash -c 'sleep 0.5; gnome-terminal --tab -- $0 $@'" >
        <param name="model_file" type="string" value="$(find prometheus_detection_circlex)/spire_caffe/examples/spire_x_classification/xnet_deploy.prototxt" />
        <param name="trained_file" type="string" value="$(find prometheus_detection_circlex)/spire_caffe/examples/spire_x_classification/xnet_iter_10000.caffemodel" />
        <param name="label_file" type="string" value="$(find prometheus_detection_circlex)/spire_caffe/examples/spire_x_classification/label.txt" />
    </node>

    <node pkg="prometheus_gimbal_control" type="gimbal_control" name="gimbal_control_server" output="screen" launch-prefix="bash -c 'sleep 1; gnome-terminal --tab -- $0 $@'"></node>

	<!-- 启动MAVROS -->
	<!-- 不同机载电脑,注意修改fcu_url至正确的端口号及波特率 -->
	<node pkg="mavros" type="mavros_node" name="mavros" output="screen">
		<param name="fcu_url" value="/dev/ttyTHS0:921600" />
		<!--param name="gcs_url" value="udp://@192.168.31.46" / -->
		<param name="gcs_url" value="" />
		<param name="target_system_id" value="1" />
		<param name="target_component_id" value="1" />
		<rosparam command="load" file="$(find p600_experiment)/config/mavros_config/px4_pluginlists_gps.yaml" />
		<rosparam command="load" file="$(find p600_experiment)/config/mavros_config/px4_config_gps.yaml" />
	</node>

    <node pkg="prometheus_control" type="px4_pos_estimator" name="px4_pos_estimator" output="screen">
        <!-- 0 for vicon， 1 for 激光SLAM, 2 for gazebo ground truth(仿真用) -->
        <param name="input_source" value="9" />
        <param name="rate_hz" value="30.0" />
    </node>

    <!-- run the px4_sender.cpp -->
    <node pkg="prometheus_control" type="px4_sender" name="px4_sender" output="screen">
        <!-- <rosparam command="load" file="$(find prometheus_experiment)/config/prometheus_control_config/px4_sender.yaml" /> -->
        <param name="Takeoff_height" type="int" value="1" />
        <param name="Land_speed" type="double" value="0.1" />
        <param name="Disarm_height" type="double" value="2" />
        <param name="Land_mode" type="int" value="1" />
        <param name="geo_fence/x_min" type="double" value="-30" />
        <param name="geo_fence/x_max" type="double" value="30" />
        <param name="geo_fence/y_min" type="double" value="-30" />
        <param name="geo_fence/y_max" type="double" value="30" />
        <param name="geo_fence/z_min" type="double" value="-30.5" />
        <param name="geo_fence/z_max" type="double" value="10" />
    </node>


    <node pkg="prometheus_gimbal_control" type="gimbal_tracking" name="gimbal_tracking_circlex" output="screen" launch-prefix="bash -c 'sleep 2.5; gnome-terminal --tab -- $0 $@'"></node>

    <node pkg="prometheus_station" type="ground_station_msg" name="ground_station_msg" output="screen" launch-prefix="bash -c 'sleep 1.5; gnome-terminal --tab -- $0 $@'"></node>

    <node pkg="prometheus_station" type="ground_station" name="ground_station" output="screen" launch-prefix="bash -c 'sleep 2; gnome-terminal --tab -- $0 $@'"></node>

    <node pkg="prometheus_mission" type="circlex_landing_gimbal" name="circlex_landing" output="screen" launch-prefix="bash -c 'sleep 3.5; gnome-terminal --tab -- $0 $@'">
        <param name="default_pitch" type="double" value="-40" />
        <param name="land_move_scale" type="double" value="0.4" />
        <param name="land_high" type="double" value="0.3" />
        <param name="ignore_error_pitch" type="int" value="15" />
        <param name="max_count_vision_lost" type="int" value="10" />
        <param name="vel_smooth_scale" type="double" value="0.5" />
        <param name="vel_smooth_thresh" type="double" value="0.4" />
        <param name="object_lost_vel_weaken" type="double" value="0.8" />
        <param name="static_vel_thresh" type="double" value="0.05"/>
        <param name="max_vel" type="double" value="1"/>
    </node>

    <node pkg="rqt_image_view" type="rqt_image_view" name="rqt_image_view"></node>

</launch>

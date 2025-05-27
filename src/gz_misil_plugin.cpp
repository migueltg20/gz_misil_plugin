#include "gz_misil_plugin/gz_misil_plugin.h"

namespace gz
{

	GzMisilPlugin::GzMisilPlugin()
	{
		// Inicializar el periodo de actualización
		update_period_ = std::chrono::nanoseconds(0);
		// Initialize ROS2 clock
		ros_clock_ = rclcpp::Clock(RCL_SYSTEM_TIME);
	}

	void GzMisilPlugin::Configure(const sim::Entity& _entity, const std::shared_ptr<const sdf::Element>& _sdf,
		sim::EntityComponentManager& _ecm, sim::EventManager& _eventMgr)
	{
		// Inicializar nodo ROS2
		if (!rclcpp::ok())
		{
			rclcpp::init(0, nullptr);
		}
		node_ = std::make_shared<rclcpp::Node>("gz_misil_plugin");

		ros_spin_thread_ = std::thread([this]() {
			rclcpp::spin(this->node_);
		});

		// Obtener entidades del mundo y modelo
		world_entity_ = _ecm.EntityByComponents(sim::components::World());
		model_entity_ = _entity;

		// Verificar que las entidades son válidas
		if (world_entity_ == sim::kNullEntity)
		{
			RCLCPP_WARN(node_->get_logger(), "Misil-Plugin: World entity not found!");
		}
		if (model_entity_ == sim::kNullEntity)
		{
			RCLCPP_FATAL(node_->get_logger(), "Misil-Plugin: Model entity is null!");
			return;
		}

		// Obtener tasa de actualización
		double update_rate = _sdf->Get<double>("real_time_update_rate");
		update_period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1.0 / update_rate));
		last_update_time_ = std::chrono::steady_clock::time_point();

		use_parent_as_reference_ = true;

		RCLCPP_INFO(node_->get_logger(), "Misil-Plugin: Plugin is running.");

		// Inicializar misil
		auto name_comp = _ecm.Component<sim::components::Name>(model_entity_);
		if (name_comp)
		{
			// Asignar nombre del modelo
			misil_.model_name = name_comp->Data();
		}

		// initialize missile link
		auto linkEnt = gz::sim::Model(model_entity_).LinkByName(_ecm, "base_link");
		misil_.missileLink = gz::sim::Link(linkEnt);
		if (!misil_.missileLink.Valid(_ecm)) {
			RCLCPP_FATAL(node_->get_logger(), "Misil-Plugin: base_link not found!");
			return;
		}

		// Crear suscriptor de la posición del UAV
		std::string uav_pos_topic = "/fmu/out/vehicle_odometry";
		uav_pos_sub_ = node_->create_subscription<px4_msgs::msg::VehicleOdometry>(uav_pos_topic, rclcpp::QoS(10).best_effort(),
			[this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
				this->uavPoseCallback(msg);
			});
		RCLCPP_INFO(node_->get_logger(), "Misil-Plugin: UAV Pose Subscribing in %s", uav_pos_topic.c_str());
	}

	void GzMisilPlugin::Reset(const sim::UpdateInfo& _info, sim::EntityComponentManager& _ecm)
	{
		RCLCPP_INFO(node_->get_logger(), "Misil-Plugin: Resetting Plugin");
		last_update_time_ = std::chrono::steady_clock::now();
	}

	void GzMisilPlugin::PreUpdate(const sim::UpdateInfo& _info, sim::EntityComponentManager& _ecm)
	{
		// Verificar si es tiempo de actualizar
		auto current_time = std::chrono::steady_clock::now();
		auto elapsed = current_time - last_update_time_;
		if (elapsed < update_period_)
			return;
		last_update_time_ = current_time;
		double dt = std::chrono::duration<double>(elapsed).count();

		auto misil_pose_comp = _ecm.Component<sim::components::Pose>(model_entity_);
		auto misil_velocity_comp = _ecm.Component<sim::components::LinearVelocity>(model_entity_);
		if (misil_pose_comp)
			misil_.pose = misil_pose_comp->Data();
		if (misil_velocity_comp)
			misil_.velocity = misil_velocity_comp->Data();

		// Obtener los ángulos de Euler del misil
		tf2::Quaternion q(misil_.pose.Rot().X(), misil_.pose.Rot().Y(), misil_.pose.Rot().Z(), misil_.pose.Rot().W());
		tf2::Matrix3x3 m(q);
		double roll, pitch, current_yaw;
		m.getRPY(roll, pitch, current_yaw);

		// Convertir la pose a mensaje de tipo geometry_msgs::msg::Pose
		geometry_msgs::msg::Pose pose_msg;
		pose_msg.position.x = misil_.pose.Pos().X();
		pose_msg.position.y = misil_.pose.Pos().Y();
		pose_msg.position.z = misil_.pose.Pos().Z();
		pose_msg.orientation.x = q.x();
		pose_msg.orientation.y = q.y();
		pose_msg.orientation.z = q.z();
		pose_msg.orientation.w = q.w();

		geometry_msgs::msg::Pose uav_pos_copy;
		{
			std::lock_guard<std::mutex> lock(uav_mutex_);
			uav_pos_copy = uav_pos_;
		}

		// Pure pursuit
		double dx = uav_pos_copy.position.x - pose_msg.position.x;
		double dy = uav_pos_copy.position.y - pose_msg.position.y;
		double dz = uav_pos_copy.position.z - pose_msg.position.z;
		double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

		geometry_msgs::msg::Twist desired_twist;
		if (dist > 1e-6) {
			desired_twist.linear.x = (dx/dist) * missile_speed_module_;
			desired_twist.linear.y = (dy/dist) * missile_speed_module_;
			desired_twist.linear.z = (dz/dist) * missile_speed_module_;
		} else {
			// Very close to target: stop
			desired_twist.linear.x = desired_twist.linear.y = desired_twist.linear.z = 0.0;
		}
		// Velocidad angular (x,y,z) para apuntar al UAV
		double desired_yaw = std::atan2(dy, dx);
		double hor_dist = std::sqrt(dx * dx + dy * dy);
		double desired_pitch = std::atan2(dz, hor_dist);
		double desired_roll = 0.0; // Asumir que se desea sin inclinación

		double yaw_diff = desired_yaw - current_yaw;
		if (yaw_diff > M_PI)
		{
			yaw_diff -= 2 * M_PI; // Ajustar al rango [-pi, pi]
		}
		else if (yaw_diff < -M_PI)
		{
			yaw_diff += 2 * M_PI; // Ajustar al rango [-pi, pi]
		}

		double pitch_diff = desired_pitch - pitch;
		double roll_diff = desired_roll - roll;

		// Aplicar ganancias para el control de la velocidad angular
		desired_twist.angular.z = yaw_diff * 2.0;   // control de yaw
		desired_twist.angular.y = pitch_diff * 2.0;   // control de pitch
		desired_twist.angular.x = roll_diff * 2.0;    // control de roll

		gz::math::Vector3d direction = gz::math::Vector3d(dx, dy, dz);
		direction.Normalize();

		double ax = (desired_twist.linear.x - misil_.velocity.X()) / dt;
		double ay = (desired_twist.linear.y - misil_.velocity.Y()) / dt;
		double az = (desired_twist.linear.z - misil_.velocity.Z()) / dt;

		double Fx = ax * 13.0; // Masa del misil (13 kg)
		double Fy = ay * 13.0; 
		double Fz = az * 13.0 + 127.5; // compenso fuerza de gravedad (13 kg * 9.81 m/s^2)
		
		gz::math::Vector3d force = gz::math::Vector3d(Fx, Fy, Fz);

		misil_.missileLink.AddWorldForce(_ecm, force);
	}

	void GzMisilPlugin::uavPoseCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
	{
		std::lock_guard<std::mutex> lock(uav_mutex_);
		// Actualizar la posición del UAV
		uav_pos_.position.x = msg->position[0];
		uav_pos_.position.y = msg->position[1];
		uav_pos_.position.z = msg->position[2];

		uav_pos_.orientation.x = msg->q[0];
		uav_pos_.orientation.y = msg->q[1];
		uav_pos_.orientation.z = msg->q[2];
		uav_pos_.orientation.w = msg->q[3];

		// Transformar de NED a ENU
		std::swap(uav_pos_.position.x, uav_pos_.position.y);
		uav_pos_.position.z = -uav_pos_.position.z;
		std::swap(uav_pos_.orientation.x, uav_pos_.orientation.y);
		uav_pos_.orientation.z = -uav_pos_.orientation.z;
		// No se invierte la w
	}

	GzMisilPlugin::~GzMisilPlugin()
	{
		if (ros_spin_thread_.joinable()) {
			rclcpp::shutdown();
			ros_spin_thread_.join();
		}
	}

	// Registrar el plugin con Gazebo
	GZ_ADD_PLUGIN(
		GzMisilPlugin,
		sim::System,
		GzMisilPlugin::ISystemConfigure,
		GzMisilPlugin::ISystemPreUpdate,
		GzMisilPlugin::ISystemReset
	)

	// GZ_ADD_PLUGIN_ALIAS(gz::GzMisilPlugin, "gz_misil_plugin");

} // namespace gz


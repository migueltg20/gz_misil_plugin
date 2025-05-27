#pragma once

// Standard
#include <random>
#include <mutex>
#include <cmath>  
#include <thread>
#include <algorithm>

// ROS2
#include <rclcpp/rclcpp.hpp>

// Gazebo
// Plugin
#include <gz/plugin/Register.hh>
// Sim
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/Util.hh>
// Common
#include <gz/common/Profiler.hh>
// Math
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
// SDF
#include <sdf/Box.hh>

// TF2
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Eigen
#include <Eigen/Dense>

// Mensajes de ROS2
#include <geometry_msgs/msg/twist.hpp>

// PX4
#include <px4_msgs/msg/vehicle_odometry.hpp>

namespace gz
{

    // Clase del plugin
    class GzMisilPlugin :
        public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate,
        public gz::sim::ISystemReset
    {
    public: 
        struct Misil
        {
            gz::math::Pose3d pose;
            gz::math::Vector3d velocity;
            std::string model_name;
            gz::sim::Link missileLink;
            Misil() : pose(), velocity(), model_name(), missileLink() {}
        };

    public: // Parámetros
        bool use_parent_as_reference_;

    public: // Constructor y destructor
        GzMisilPlugin();
        ~GzMisilPlugin() override;

    public: // Métodos
        void Configure(const sim::Entity& _entity, const std::shared_ptr<const sdf::Element>& _sdf,
            sim::EntityComponentManager& _ecm, sim::EventManager& _eventMgr) override;
        void Reset(const sim::UpdateInfo& _info, sim::EntityComponentManager& _ecm) override;
        void PreUpdate(const sim::UpdateInfo& _info, sim::EntityComponentManager& _ecm) override;
        void uavPoseCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

    private: // Componentes de Gazebo/ROS2
        // Nodo ROS2
        std::shared_ptr<rclcpp::Node> node_;
        // Temporizadores
        rclcpp::Clock ros_clock_;
        std::chrono::steady_clock::duration update_period_;
        std::chrono::steady_clock::time_point last_update_time_;
        // Entidades de Gazebo
        sim::Entity world_entity_;      // Entidad del mundo
        sim::Entity model_entity_;      // Entidad del modelo

        Misil misil_; // Estructura del misil

        // Publicadores y suscriptores
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr uav_pos_sub_; // Suscriptor de posición del UAV

        geometry_msgs::msg::Pose uav_pos_; // Posición del UAV

        std::thread ros_spin_thread_;
        std::mutex uav_mutex_;

    private: // Constantes
        // Módulo de la velocidad del misil
        const double missile_speed_module_ = 10.0; // Velocidad del misil en m/s
    };

} // namespace gz
<template>
  <LessonContainer>
    <!-- ========================================================================================
         HERO SECTION: THE PLUGIN MATRIX
         Visual: Plugin nodes connecting to Gazebo Core
         ======================================================================================== -->
    <div class="hero-section text-white q-mb-xl">
      <div class="hero-content">
        <div class="hero-badge"><q-icon name="extension" /> MÓDULO 6.4: GAZEBO PLUGINS</div>
        <h1 class="hero-title">
          Gazebo Plugins: <span class="gradient-text">Extending the Simulation</span>
        </h1>
        <p class="hero-subtitle">
          Los plugins son código C++ que se ejecuta <strong>dentro</strong> del proceso de Gazebo,
          sin latencia de red. Puedes crear sensores personalizados, actuadores complejos, o
          modificar la física en tiempo real. Aquí aprenderás a escribir plugins de nivel
          industrial.
        </p>

        <div class="hero-stats">
          <div class="stat-item">
            <div class="stat-val">4 Tipos</div>
            <div class="stat-label">Plugin Classes</div>
          </div>
          <div class="stat-item">
            <div class="stat-val">&lt;1ms</div>
            <div class="stat-label">Latency Overhead</div>
          </div>
          <div class="stat-item">
            <div class="stat-val">C++17</div>
            <div class="stat-label">Language</div>
          </div>
        </div>
      </div>

      <div class="hero-viz">
        <!-- Plugin Connection Animation -->
        <svg viewBox="0 0 200 200" class="plugin-matrix">
          <defs>
            <radialGradient id="coreGrad">
              <stop offset="0%" stop-color="#fbbf24" stop-opacity="1" />
              <stop offset="100%" stop-color="#f59e0b" stop-opacity="0.3" />
            </radialGradient>
          </defs>

          <!-- Gazebo Core -->
          <circle cx="100" cy="100" r="20" fill="url(#coreGrad)" class="core-node" />
          <text x="100" y="105" text-anchor="middle" fill="#fff" font-size="8" font-weight="700">
            CORE
          </text>

          <!-- Plugin Nodes -->
          <g class="plugin-node" data-type="model">
            <circle cx="50" cy="50" r="12" fill="#3b82f6" opacity="0.8" />
            <line
              x1="100"
              y1="100"
              x2="50"
              y2="50"
              stroke="#3b82f6"
              stroke-width="2"
              class="connection c1"
            />
            <text x="50" y="35" text-anchor="middle" fill="#3b82f6" font-size="7">Model</text>
          </g>

          <g class="plugin-node" data-type="sensor">
            <circle cx="150" cy="50" r="12" fill="#22c55e" opacity="0.8" />
            <line
              x1="100"
              y1="100"
              x2="150"
              y2="50"
              stroke="#22c55e"
              stroke-width="2"
              class="connection c2"
            />
            <text x="150" y="35" text-anchor="middle" fill="#22c55e" font-size="7">Sensor</text>
          </g>

          <g class="plugin-node" data-type="world">
            <circle cx="50" cy="150" r="12" fill="#a855f7" opacity="0.8" />
            <line
              x1="100"
              y1="100"
              x2="50"
              y2="150"
              stroke="#a855f7"
              stroke-width="2"
              class="connection c3"
            />
            <text x="50" y="170" text-anchor="middle" fill="#a855f7" font-size="7">World</text>
          </g>

          <g class="plugin-node" data-type="visual">
            <circle cx="150" cy="150" r="12" fill="#ef4444" opacity="0.8" />
            <line
              x1="100"
              y1="100"
              x2="150"
              y2="150"
              stroke="#ef4444"
              stroke-width="2"
              class="connection c4"
            />
            <text x="150" y="170" text-anchor="middle" fill="#ef4444" font-size="7">Visual</text>
          </g>
        </svg>
      </div>
    </div>

    <!-- ========================================================================================
         BLOCK A: PLUGIN ARCHITECTURE & LIFECYCLE
         Depth: Software Architecture & Memory Management
         ======================================================================================== -->
    <div class="content-block">
      <SectionTitle>1. Anatomía de un Plugin</SectionTitle>

      <TextBlock>
        Un plugin de Gazebo es una biblioteca dinámica (.so en Linux, .dll en Windows) que Gazebo
        carga en tiempo de ejecución. A diferencia de un nodo de ROS 2, el plugin corre en el
        <strong>mismo proceso</strong>
        que Gazebo, lo que elimina la latencia de serialización y red.
      </TextBlock>

      <!-- PLUGIN TYPES GRID -->
      <div class="plugin-types-grid q-my-lg">
        <div class="type-card model-type">
          <q-icon name="view_in_ar" size="xl" />
          <div class="type-title">Model Plugin</div>
          <div class="type-desc">
            Se adjunta a un <code>&lt;model&gt;</code>. Acceso a links, joints, sensores del modelo.
          </div>
          <div class="type-use">
            <strong>Uso:</strong> Actuadores custom, controladores de robots.
          </div>
        </div>

        <div class="type-card sensor-type">
          <q-icon name="sensors" size="xl" />
          <div class="type-title">Sensor Plugin</div>
          <div class="type-desc">
            Se adjunta a un <code>&lt;sensor&gt;</code>. Procesa datos de sensores o crea sensores
            nuevos.
          </div>
          <div class="type-use"><strong>Uso:</strong> Sensores IR, térmicos, químicos.</div>
        </div>

        <div class="type-card world-type">
          <q-icon name="public" size="xl" />
          <div class="type-title">World Plugin</div>
          <div class="type-desc">
            Se adjunta al <code>&lt;world&gt;</code>. Acceso global a todos los modelos y física.
          </div>
          <div class="type-use">
            <strong>Uso:</strong> Spawn dinámico, control de tiempo, telemetría.
          </div>
        </div>

        <div class="type-card visual-type">
          <q-icon name="visibility" size="xl" />
          <div class="type-title">Visual Plugin</div>
          <div class="type-desc">
            Se adjunta a un <code>&lt;visual&gt;</code>. Modifica apariencia en tiempo real.
          </div>
          <div class="type-use"><strong>Uso:</strong> LEDs, pantallas, efectos visuales.</div>
        </div>
      </div>

      <!-- LIFECYCLE DIAGRAM -->
      <div class="lifecycle-container q-mt-xl">
        <div class="lifecycle-header">
          <q-icon name="autorenew" />
          <span>Plugin Lifecycle</span>
        </div>
        <div class="lifecycle-flow">
          <div class="lifecycle-stage load">
            <div class="stage-number">1</div>
            <div class="stage-name">Load()</div>
            <div class="stage-desc">
              Gazebo carga el plugin. Aquí lees parámetros del SDF y reservas memoria.
            </div>
          </div>
          <div class="flow-arrow">→</div>
          <div class="lifecycle-stage init">
            <div class="stage-number">2</div>
            <div class="stage-name">Init()</div>
            <div class="stage-desc">
              Inicialización completa. Conectas a eventos y creas publishers de ROS 2.
            </div>
          </div>
          <div class="flow-arrow">→</div>
          <div class="lifecycle-stage update">
            <div class="stage-number">3</div>
            <div class="stage-name">Update()</div>
            <div class="stage-desc">
              Llamado cada frame de simulación (~1000 Hz). Aquí va tu lógica principal.
            </div>
          </div>
          <div class="flow-arrow">→</div>
          <div class="lifecycle-stage fini">
            <div class="stage-number">4</div>
            <div class="stage-name">Fini()</div>
            <div class="stage-desc">Limpieza. Liberas memoria y desconectas eventos.</div>
          </div>
        </div>
      </div>

      <!-- MINIMAL PLUGIN CODE -->
      <div class="code-section q-mt-xl">
        <div class="code-header">
          <q-icon name="code" />
          <span>MinimalModelPlugin.cpp (Template Base)</span>
        </div>
        <CodeBlock
          lang="cpp"
          :copyable="true"
          title="Estructura Mínima de un Model Plugin"
          content='#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo {
  class MinimalModelPlugin : public ModelPlugin {

    // ==================== LIFECYCLE HOOKS ====================

    /// Called when the plugin is loaded (SDF parsing)
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override {
      // Store model pointer
      this->model_ = _model;

      // Read SDF parameters
      if (_sdf->HasElement("update_rate")) {
        this->update_rate_ = _sdf->Get<double>("update_rate");
      }

      gzdbg << "MinimalModelPlugin loaded for model: "
            << this->model_->GetName() << std::endl;

      // Connect to the world update event
      this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&MinimalModelPlugin::OnUpdate, this)
      );
    }

    /// Called every simulation step
    private: void OnUpdate() {
      // Get current simulation time
      common::Time sim_time = this->model_->GetWorld()->SimTime();

      // Example: Print every 1 second
      if ((sim_time - this->last_print_time_).Double() >= 1.0) {
        gzmsg << "Simulation time: " << sim_time.Double() << "s" << std::endl;
        this->last_print_time_ = sim_time;
      }
    }

    // ==================== MEMBER VARIABLES ====================

    private: physics::ModelPtr model_;           ///< Pointer to the model
    private: event::ConnectionPtr update_connection_; ///< Event connection
    private: double update_rate_ = 1000.0;       ///< Hz
    private: common::Time last_print_time_;      ///< For throttling output
  };

  // Register this plugin with Gazebo
  GZ_REGISTER_MODEL_PLUGIN(MinimalModelPlugin)
}

/*
 * USAGE IN SDF:
 * <model name="my_robot">
 *   <plugin name="minimal_plugin" filename="libMinimalModelPlugin.so">
 *     <update_rate>500</update_rate>
 *   </plugin>
 * </model>
 */'
        />
      </div>
    </div>

    <!-- SECTION 2: GAZEBO API ACCESS -->
    <div class="content-block q-mt-xl">
      <SectionTitle>2. Acceso a la API de Gazebo</SectionTitle>

      <TextBlock>
        Desde un plugin, tienes acceso directo a la API interna de Gazebo. Esto es poderoso pero
        peligroso: Gazebo usa <strong>múltiples threads</strong>, y acceder a objetos compartidos
        sin sincronización causa race conditions.
      </TextBlock>

      <!-- API ACCESS VIZ -->
      <div class="api-access-grid q-my-lg">
        <div class="api-card">
          <div class="api-header">
            <q-icon name="link" color="blue" />
            <span>Links & Joints</span>
          </div>
          <div class="api-content">
            <code>model->GetLink("wheel_left")</code>
            <div class="api-desc">Acceso a propiedades físicas: pose, velocidad, masa.</div>
            <code>joint->SetForce(0, 10.0)</code>
            <div class="api-desc">Aplicar torque/fuerza a un joint.</div>
          </div>
        </div>

        <div class="api-card">
          <div class="api-header">
            <q-icon name="public" color="purple" />
            <span>World Access</span>
          </div>
          <div class="api-content">
            <code>world->ModelByName("robot")</code>
            <div class="api-desc">Buscar modelos por nombre.</div>
            <code>world->InsertModelFile("model://box")</code>
            <div class="api-desc">Spawn dinámico de modelos.</div>
          </div>
        </div>

        <div class="api-card">
          <div class="api-header">
            <q-icon name="schedule" color="green" />
            <span>Time & Events</span>
          </div>
          <div class="api-content">
            <code>world->SimTime()</code>
            <div class="api-desc">Tiempo de simulación (no real-time).</div>
            <code>Events::ConnectWorldUpdateBegin()</code>
            <div class="api-desc">Hook al loop de física.</div>
          </div>
        </div>
      </div>

      <!-- THREAD SAFETY CODE -->
      <div class="code-section">
        <div class="code-header">
          <q-icon name="code" />
          <span>ThreadSafeJointAccess.cpp (Sincronización con Mutex)</span>
        </div>
        <CodeBlock
          lang="cpp"
          :copyable="true"
          title="Acceso Seguro a Joints desde Múltiples Threads"
          content='#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <mutex>

namespace gazebo {
  class ThreadSafePlugin : public ModelPlugin {

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override {
      this->model_ = _model;
      this->joint_ = this->model_->GetJoint("revolute_joint");

      // Connect to update event (runs in physics thread)
      this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ThreadSafePlugin::OnUpdate, this)
      );

      // Start a separate thread for ROS 2 callbacks
      this->ros_thread_ = std::thread(&ThreadSafePlugin::ROSSpinThread, this);
    }

    /// Called from PHYSICS THREAD
    private: void OnUpdate() {
      std::lock_guard<std::mutex> lock(this->joint_mutex_);

      // Safe to access joint here
      double current_angle = this->joint_->Position(0);

      // Apply control
      double error = this->target_angle_ - current_angle;
      double force = this->kp_ * error;
      this->joint_->SetForce(0, force);
    }

    /// Called from ROS 2 THREAD
    private: void ROSSpinThread() {
      // Simulate ROS 2 callback setting target
      while (this->running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        {
          std::lock_guard<std::mutex> lock(this->joint_mutex_);
          this->target_angle_ += 0.1; // Increment target
        }
      }
    }

    public: ~ThreadSafePlugin() {
      this->running_ = false;
      if (this->ros_thread_.joinable()) {
        this->ros_thread_.join();
      }
    }

    private: physics::ModelPtr model_;
    private: physics::JointPtr joint_;
    private: event::ConnectionPtr update_connection_;

    // Thread synchronization
    private: std::mutex joint_mutex_;
    private: std::thread ros_thread_;
    private: std::atomic<bool> running_{true};

    // Shared state (protected by mutex)
    private: double target_angle_ = 0.0;
    private: double kp_ = 10.0;
  };

  GZ_REGISTER_MODEL_PLUGIN(ThreadSafePlugin)
}

/*
 * KEY POINTS:
 * 1. Gazebo physics runs in its own thread.
 * 2. ROS 2 callbacks run in separate threads.
 * 3. ALWAYS use mutex when accessing shared state.
 * 4. Prefer std::atomic for simple flags.
 */'
        />
      </div>
    </div>

    <!-- ========================================================================================
         BLOCK B: MODEL PLUGINS (CUSTOM ACTUATORS)
         Depth: Control Systems & Dynamics
         ======================================================================================== -->
    <div class="section-divider">
      <q-separator spaced="lg" color="indigo-9" />
    </div>

    <!-- SECTION 3: LINEAR ACTUATOR PLUGIN -->
    <div class="content-block">
      <SectionTitle>3. Model Plugin: Actuador Lineal con PID</SectionTitle>

      <TextBlock>
        Un actuador lineal (como un elevador hidráulico) requiere control de posición preciso.
        Implementaremos un <strong>controlador PID</strong> que corre a 1000 Hz dentro de Gazebo,
        sin latencia de ROS 2.
      </TextBlock>

      <!-- PID THEORY VIZ -->
      <div class="pid-viz-container q-my-lg">
        <div class="pid-diagram">
          <div class="pid-block setpoint">
            <div class="block-label">Setpoint</div>
            <code>r(t)</code>
          </div>
          <div class="pid-arrow">→</div>
          <div class="pid-block error">
            <div class="block-label">Error</div>
            <code>e(t) = r - y</code>
          </div>
          <div class="pid-arrow">→</div>
          <div class="pid-block controller">
            <div class="block-label">PID Controller</div>
            <code>u(t) = Kp·e + Ki·∫e + Kd·ė</code>
          </div>
          <div class="pid-arrow">→</div>
          <div class="pid-block plant">
            <div class="block-label">Actuator</div>
            <code>SetForce(u)</code>
          </div>
          <div class="pid-arrow feedback">↓</div>
          <div class="pid-block output">
            <div class="block-label">Position</div>
            <code>y(t)</code>
          </div>
        </div>
      </div>

      <!-- LINEAR ACTUATOR CODE -->
      <q-expansion-item
        class="bg-slate-800 text-white rounded-lg border border-slate-700 q-mb-xl"
        icon="code"
        label="Ver Código Completo: LinearActuatorPlugin.cpp"
        header-class="text-amber-400 font-bold"
      >
        <div class="q-pa-md">
          <CodeBlock
            lang="cpp"
            content='#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace gazebo {
  class LinearActuatorPlugin : public ModelPlugin {

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override {
      this->model_ = _model;
      this->joint_ = this->model_->GetJoint("prismatic_joint");

      // Read PID gains from SDF
      if (_sdf->HasElement("kp")) this->kp_ = _sdf->Get<double>("kp");
      if (_sdf->HasElement("ki")) this->ki_ = _sdf->Get<double>("ki");
      if (_sdf->HasElement("kd")) this->kd_ = _sdf->Get<double>("kd");

      // Initialize ROS 2
      if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
      }
      this->ros_node_ = rclcpp::Node::make_shared("linear_actuator_plugin");

      // Subscribe to position commands
      this->cmd_sub_ = this->ros_node_->create_subscription<std_msgs::msg::Float64>(
        "/actuator/cmd_position", 10,
        std::bind(&LinearActuatorPlugin::OnCmdPosition, this, std::placeholders::_1)
      );

      // Publish current position
      this->pos_pub_ = this->ros_node_->create_publisher<std_msgs::msg::Float64>(
        "/actuator/position", 10
      );

      // Connect to update event
      this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&LinearActuatorPlugin::OnUpdate, this)
      );

      // Start ROS 2 spin thread
      this->ros_thread_ = std::thread([this]() {
        rclcpp::spin(this->ros_node_);
      });

      gzmsg << "LinearActuatorPlugin loaded. PID: Kp=" << this->kp_
            << " Ki=" << this->ki_ << " Kd=" << this->kd_ << std::endl;
    }

    private: void OnCmdPosition(const std_msgs::msg::Float64::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->target_position_ = msg->data;
      this->integral_error_ = 0.0; // Reset integral on new command
    }

    private: void OnUpdate() {
      std::lock_guard<std::mutex> lock(this->mutex_);

      // Get current position
      double current_pos = this->joint_->Position(0);

      // Calculate error
      double error = this->target_position_ - current_pos;

      // PID calculation
      this->integral_error_ += error * this->dt_;
      double derivative_error = (error - this->last_error_) / this->dt_;

      double control_output =
        this->kp_ * error +
        this->ki_ * this->integral_error_ +
        this->kd_ * derivative_error;

      // Apply force
      this->joint_->SetForce(0, control_output);

      // Update state
      this->last_error_ = error;

      // Publish position (throttled to 10 Hz)
      auto now = this->model_->GetWorld()->SimTime();
      if ((now - this->last_pub_time_).Double() >= 0.1) {
        auto msg = std_msgs::msg::Float64();
        msg.data = current_pos;
        this->pos_pub_->publish(msg);
        this->last_pub_time_ = now;
      }
    }

    public: ~LinearActuatorPlugin() {
      rclcpp::shutdown();
      if (this->ros_thread_.joinable()) {
        this->ros_thread_.join();
      }
    }

    private: physics::ModelPtr model_;
    private: physics::JointPtr joint_;
    private: event::ConnectionPtr update_connection_;

    // ROS 2
    private: rclcpp::Node::SharedPtr ros_node_;
    private: rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cmd_sub_;
    private: rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pos_pub_;
    private: std::thread ros_thread_;

    // PID parameters
    private: double kp_ = 100.0;
    private: double ki_ = 10.0;
    private: double kd_ = 5.0;

    // State
    private: double target_position_ = 0.0;
    private: double last_error_ = 0.0;
    private: double integral_error_ = 0.0;
    private: double dt_ = 0.001; // 1 kHz
    private: common::Time last_pub_time_;

    // Thread safety
    private: std::mutex mutex_;
  };

  GZ_REGISTER_MODEL_PLUGIN(LinearActuatorPlugin)
}

/*
 * USAGE IN SDF:
 * <plugin name="actuator" filename="libLinearActuatorPlugin.so">
 *   <kp>150.0</kp>
 *   <ki>20.0</ki>
 *   <kd>8.0</kd>
 * </plugin>
 */'
            :copyable="true"
          />
        </div>
      </q-expansion-item>
    </div>

    <!-- SECTION 4: MAGNETIC GRIPPER PLUGIN -->
    <div class="content-block q-mt-xl">
      <SectionTitle>4. Model Plugin: Gripper Magnético</SectionTitle>

      <TextBlock>
        Un gripper magnético detecta objetos cercanos y crea un <strong>joint dinámico</strong> para
        "pegarlos". Esto es más eficiente que aplicar fuerzas constantemente.
      </TextBlock>

      <div class="code-section">
        <div class="code-header">
          <q-icon name="code" />
          <span>MagneticGripperPlugin.cpp (Dynamic Joint Creation)</span>
        </div>
        <CodeBlock
          lang="cpp"
          :copyable="true"
          title="Gripper que Crea Joints en Runtime"
          content='#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo {
  class MagneticGripperPlugin : public ModelPlugin {

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override {
      this->model_ = _model;
      this->link_ = this->model_->GetLink("gripper_link");
      this->world_ = this->model_->GetWorld();

      // Read detection radius
      if (_sdf->HasElement("radius")) {
        this->detection_radius_ = _sdf->Get<double>("radius");
      }

      this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&MagneticGripperPlugin::OnUpdate, this)
      );
    }

    private: void OnUpdate() {
      if (this->is_gripping_) {
        // Check if object still exists
        if (!this->gripped_model_ || this->gripped_model_->GetWorld() == nullptr) {
          this->ReleaseObject();
        }
        return;
      }

      // Search for nearby objects
      auto gripper_pose = this->link_->WorldPose();
      auto models = this->world_->Models();

      for (auto& model : models) {
        if (model == this->model_) continue; // Skip self

        auto model_pose = model->WorldPose();
        double distance = gripper_pose.Pos().Distance(model_pose.Pos());

        if (distance < this->detection_radius_) {
          this->GripObject(model);
          break;
        }
      }
    }

    private: void GripObject(physics::ModelPtr object) {
      gzmsg << "Gripping object: " << object->GetName() << std::endl;

      // Create a fixed joint between gripper and object
      this->grip_joint_ = this->world_->Physics()->CreateJoint(
        "fixed", this->model_
      );

      this->grip_joint_->Attach(
        this->link_,
        object->GetLink() // Attach to first link of object
      );

      this->grip_joint_->Load(this->link_, object->GetLink(), ignition::math::Pose3d());
      this->grip_joint_->Init();

      this->gripped_model_ = object;
      this->is_gripping_ = true;
    }

    public: void ReleaseObject() {
      if (this->grip_joint_) {
        this->grip_joint_->Detach();
        this->grip_joint_.reset();
      }
      this->gripped_model_.reset();
      this->is_gripping_ = false;
      gzmsg << "Object released" << std::endl;
    }

    private: physics::ModelPtr model_;
    private: physics::LinkPtr link_;
    private: physics::WorldPtr world_;
    private: physics::JointPtr grip_joint_;
    private: physics::ModelPtr gripped_model_;
    private: event::ConnectionPtr update_connection_;

    private: double detection_radius_ = 0.1; // 10cm
    private: bool is_gripping_ = false;
  };

  GZ_REGISTER_MODEL_PLUGIN(MagneticGripperPlugin)
}'
        />
      </div>
    </div>

    <!-- ========================================================================================
         BLOCK C: SENSOR PLUGINS (CUSTOM SENSORS)
         Depth: Signal Processing & Data Fusion
         ======================================================================================== -->
    <div class="section-divider">
      <q-separator spaced="lg" color="indigo-9" />
    </div>

    <!-- SECTION 5: IR PROXIMITY SENSOR -->
    <div class="content-block">
      <SectionTitle>5. Sensor Plugin: Proximidad Infrarroja</SectionTitle>

      <TextBlock>
        Los sensores IR (Sharp GP2Y0A21YK) miden distancia usando triangulación óptica.
        Implementaremos un sensor custom con ray casting manual y curva de respuesta no-lineal.
      </TextBlock>

      <div class="code-section">
        <div class="code-header">
          <q-icon name="code" />
          <span>IRProximitySensor.cpp (Custom Ray Casting)</span>
        </div>
        <CodeBlock
          lang="cpp"
          :copyable="true"
          title="Sensor IR con Respuesta No-Lineal"
          content='#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

namespace gazebo {
  class IRProximitySensor : public SensorPlugin {

    public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override {
      this->parent_sensor_ = _sensor;
      this->world_ = physics::get_world();

      // Get sensor pose
      this->sensor_pose_ = this->parent_sensor_->Pose();

      // Initialize ROS 2
      if (!rclcpp::ok()) rclcpp::init(0, nullptr);
      this->ros_node_ = rclcpp::Node::make_shared("ir_sensor");
      this->pub_ = this->ros_node_->create_publisher<sensor_msgs::msg::Range>(
        "/ir/range", 10
      );

      // Connect to sensor update
      this->update_connection_ = this->parent_sensor_->ConnectUpdated(
        std::bind(&IRProximitySensor::OnUpdate, this)
      );
      this->parent_sensor_->SetActive(true);
    }

    private: void OnUpdate() {
      // Perform ray cast
      auto ray_start = this->sensor_pose_.Pos();
      auto ray_dir = this->sensor_pose_.Rot().RotateVector(
        ignition::math::Vector3d(1, 0, 0) // Forward
      );
      auto ray_end = ray_start + ray_dir * this->max_range_;

      // Cast ray
      std::string entity_name;
      double distance;
      this->world_->Physics()->GetRayIntersection(
        ray_start, ray_end, distance, entity_name
      );

      // Apply IR sensor response curve (non-linear)
      double measured_distance = this->ApplyIRResponse(distance);

      // Publish ROS 2 message
      auto msg = sensor_msgs::msg::Range();
      msg.header.stamp = this->ros_node_->now();
      msg.header.frame_id = "ir_link";
      msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
      msg.field_of_view = 0.1; // ~6 degrees
      msg.min_range = 0.1;
      msg.max_range = 0.8;
      msg.range = measured_distance;

      this->pub_->publish(msg);
    }

    /// Simulate Sharp GP2Y0A21YK response curve
    private: double ApplyIRResponse(double true_distance) {
      if (true_distance < 0.1 || true_distance > 0.8) {
        return std::numeric_limits<double>::infinity();
      }

      // Inverse square law approximation + noise
      double voltage = 27.0 / (true_distance * 100.0); // Simplified
      double measured = 27.0 / voltage / 100.0;

      // Add Gaussian noise
      std::normal_distribution<double> noise(0.0, 0.01);
      measured += noise(this->rng_);

      return measured;
    }

    private: sensors::SensorPtr parent_sensor_;
    private: physics::WorldPtr world_;
    private: ignition::math::Pose3d sensor_pose_;
    private: event::ConnectionPtr update_connection_;

    private: rclcpp::Node::SharedPtr ros_node_;
    private: rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_;

    private: double max_range_ = 0.8;
    private: std::mt19937 rng_{std::random_device{}()};
  };

  GZ_REGISTER_SENSOR_PLUGIN(IRProximitySensor)
}'
        />
      </div>
    </div>

    <!-- SECTION 6: THERMAL SENSOR -->
    <div class="content-block q-mt-xl">
      <SectionTitle>6. Sensor Plugin: Temperatura Simulada</SectionTitle>

      <TextBlock>
        Simularemos un campo de temperatura basado en la distancia a fuentes de calor (modelos
        marcados). Útil para robots de inspección en plantas industriales.
      </TextBlock>

      <div class="code-section">
        <div class="code-header">
          <q-icon name="code" />
          <span>ThermalSensor.cpp (Scalar Field Interpolation)</span>
        </div>
        <CodeBlock
          lang="cpp"
          :copyable="true"
          title="Sensor de Temperatura con Campo Escalar"
          content='#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace gazebo {
  class ThermalSensor : public SensorPlugin {

    public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override {
      this->parent_sensor_ = _sensor;
      this->world_ = physics::get_world();

      // Find heat sources (models with "heat_source" tag)
      this->FindHeatSources();

      // ROS 2 setup
      if (!rclcpp::ok()) rclcpp::init(0, nullptr);
      this->ros_node_ = rclcpp::Node::make_shared("thermal_sensor");
      this->pub_ = this->ros_node_->create_publisher<std_msgs::msg::Float64>(
        "/thermal/temperature", 10
      );

      this->update_connection_ = this->parent_sensor_->ConnectUpdated(
        std::bind(&ThermalSensor::OnUpdate, this)
      );
      this->parent_sensor_->SetActive(true);
    }

    private: void FindHeatSources() {
      auto models = this->world_->Models();
      for (auto& model : models) {
        // Check if model has "temperature" property in SDF
        auto sdf = model->GetSDF();
        if (sdf->HasElement("temperature")) {
          double temp = sdf->Get<double>("temperature");
          this->heat_sources_.push_back({model, temp});
          gzmsg << "Found heat source: " << model->GetName()
                << " at " << temp << "°C" << std::endl;
        }
      }
    }

    private: void OnUpdate() {
      auto sensor_pos = this->parent_sensor_->Pose().Pos();

      // Calculate temperature based on distance to heat sources
      double ambient_temp = 20.0; // °C
      double total_temp = ambient_temp;

      for (auto& [source_model, source_temp] : this->heat_sources_) {
        auto source_pos = source_model->WorldPose().Pos();
        double distance = sensor_pos.Distance(source_pos);

        // Inverse square law for heat radiation
        if (distance > 0.1) {
          double heat_contribution = (source_temp - ambient_temp) / (distance * distance);
          total_temp += heat_contribution;
        }
      }

      // Publish temperature
      auto msg = std_msgs::msg::Float64();
      msg.data = total_temp;
      this->pub_->publish(msg);
    }

    private: sensors::SensorPtr parent_sensor_;
    private: physics::WorldPtr world_;
    private: event::ConnectionPtr update_connection_;

    private: rclcpp::Node::SharedPtr ros_node_;
    private: rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;

    private: std::vector<std::pair<physics::ModelPtr, double>> heat_sources_;
  };

  GZ_REGISTER_SENSOR_PLUGIN(ThermalSensor)
}'
        />
      </div>
    </div>

    <!-- ========================================================================================
         BLOCK D: CASE STUDY & SUMMARY
         Depth: Integration & Best Practices
         ======================================================================================== -->
    <div class="section-divider">
      <q-separator spaced="lg" color="indigo-9" />
    </div>

    <!-- SECTION 7: CASE STUDY -->
    <div class="content-block">
      <SectionTitle>7. Case Study: Robot Inspector</SectionTitle>
      <TextBlock>
        Integramos 3 plugins custom en un robot de inspección industrial: actuador telescópico,
        sensor ultrasónico custom, y logger de telemetría global.
      </TextBlock>

      <div class="case-metrics q-my-lg">
        <div class="metric-card">
          <q-icon name="speed" size="lg" color="green" />
          <div class="metric-val">0.3ms</div>
          <div class="metric-label">Plugin Overhead</div>
        </div>
        <div class="metric-card">
          <q-icon name="memory" size="lg" color="blue" />
          <div class="metric-val">12 MB</div>
          <div class="metric-label">Memory Usage</div>
        </div>
        <div class="metric-card">
          <q-icon name="update" size="lg" color="purple" />
          <div class="metric-val">1000 Hz</div>
          <div class="metric-label">Update Rate</div>
        </div>
      </div>
    </div>

    <!-- SECTION 8: DOCTORAL CHALLENGE -->
    <div class="content-block q-mb-xl">
      <SectionTitle>8. Doctor's Challenge: The Thread Race Condition</SectionTitle>

      <div class="challenge-card">
        <div class="challenge-header">
          <q-icon name="bug_report" class="c-icon" />
          <div class="c-info">
            <div class="c-title">🐛 Race Condition Mortal</div>
            <div class="c-desc">
              Tu plugin funciona perfectamente durante 5 minutos, luego Gazebo crashea con segfault.
              El backtrace muestra que crashea en <code>joint->Position()</code>. ¿Qué está mal?
            </div>
          </div>
        </div>

        <div class="challenge-options">
          <div class="c-option correct">
            <div class="opt-radio">A</div>
            <div class="opt-text">
              Estás accediendo al joint desde un callback de ROS 2 sin mutex, mientras el physics
              thread también lo usa.
              <div class="opt-feedback">
                ¡Correcto! SIEMPRE usa <code>std::lock_guard</code> cuando accedas a objetos de
                Gazebo desde threads externos.
              </div>
            </div>
          </div>
          <div class="c-option">
            <div class="opt-radio">B</div>
            <div class="opt-text">Gazebo tiene un bug en la versión que usas.</div>
          </div>
          <div class="c-option">
            <div class="opt-radio">C</div>
            <div class="opt-text">El joint fue eliminado del mundo.</div>
          </div>
        </div>
      </div>
    </div>

    <!-- SECTION 9: VIDEO TUTORIAL -->
    <div class="content-block q-mb-xl">
      <SectionTitle>9. Video Tutorial: Compilación y Debug</SectionTitle>
      <div class="video-container">
        <div class="video-wrapper">
          <iframe
            width="100%"
            height="100%"
            src="https://www.youtube.com/embed/dQw4w9WgXcQ"
            title="Gazebo Plugins Tutorial"
            frameborder="0"
            allow="
              accelerometer;
              autoplay;
              clipboard-write;
              encrypted-media;
              gyroscope;
              picture-in-picture;
            "
            allowfullscreen
          ></iframe>
        </div>
        <div class="video-caption">
          <q-icon name="live_tv" />
          Compilación con CMake, debug con GDB, y profiling con Valgrind.
        </div>
      </div>
    </div>

    <!-- SECTION 10: SUMMARY -->
    <div class="section-group q-mb-xl">
      <SectionTitle>📝 Resumen de Ingeniería</SectionTitle>
      <div class="summary-grid">
        <div class="summary-item">
          <code>Plugin Lifecycle</code>
          <span>Load() → Init() → Update() → Fini(). Update() corre a frecuencia de física.</span>
        </div>
        <div class="summary-item">
          <code>Thread Safety</code>
          <span>SIEMPRE usa mutex para acceder a Gazebo API desde ROS 2 callbacks.</span>
        </div>
        <div class="summary-item">
          <code>Model Plugin</code>
          <span>Acceso a links/joints. Ideal para actuadores y controladores.</span>
        </div>
        <div class="summary-item">
          <code>Sensor Plugin</code>
          <span>Procesa datos de sensores o crea sensores custom con ray casting.</span>
        </div>
        <div class="summary-item">
          <code>Dynamic Joints</code>
          <span>Crear/destruir joints en runtime con <code>CreateJoint()</code>.</span>
        </div>
        <div class="summary-item">
          <code>Performance</code>
          <span>Plugins añaden &lt;1ms overhead. Evita operaciones pesadas en Update().</span>
        </div>
      </div>

      <AlertBlock type="success" title="Best Practices: Plugin Development" class="q-mt-lg">
        ✅ <strong>Memory Leaks:</strong> Usa smart pointers (<code>std::shared_ptr</code>) para
        objetos ROS 2.
        <br />
        ✅ <strong>Logging:</strong> Usa <code>gzmsg</code> (info), <code>gzwarn</code> (warning),
        <code>gzerr</code> (error).
        <br />
        ✅ <strong>Testing:</strong> Corre con <code>valgrind --leak-check=full</code> para detectar
        leaks.
        <br />
        ✅ <strong>Versioning:</strong> Verifica compatibilidad de Gazebo API con
        <code>GAZEBO_MAJOR_VERSION</code>.
      </AlertBlock>
    </div>
  </LessonContainer>
</template>

<script setup lang="ts">
import LessonContainer from 'components/content/LessonContainer.vue';
import TextBlock from 'components/content/TextBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
</script>

<style scoped>
/* HERO SECTION */
.hero-section {
  background:
    radial-gradient(circle at 80% 20%, rgba(59, 130, 246, 0.15), transparent 50%),
    linear-gradient(135deg, #0f172a 0%, #1e293b 100%);
  border-radius: 24px;
  padding: 4rem 3rem;
  border: 1px solid rgba(148, 163, 184, 0.1);
  box-shadow: 0 20px 50px -10px rgba(0, 0, 0, 0.5);
  display: flex;
  justify-content: space-between;
  align-items: center;
  gap: 4rem;
  overflow: hidden;
  position: relative;
}

.hero-content {
  flex: 1;
  z-index: 2;
}

.hero-badge {
  display: inline-flex;
  align-items: center;
  gap: 0.5rem;
  background: rgba(59, 130, 246, 0.15);
  color: #60a5fa;
  padding: 0.5rem 1rem;
  border-radius: 99px;
  font-size: 0.85rem;
  font-weight: 700;
  border: 1px solid rgba(59, 130, 246, 0.3);
  margin-bottom: 1.5rem;
}

.hero-title {
  font-size: 3.5rem;
  font-weight: 800;
  line-height: 1.1;
  margin-bottom: 1.5rem;
  letter-spacing: -1px;
}

.gradient-text {
  background: linear-gradient(to right, #60a5fa, #22d3ee);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
}

.hero-subtitle {
  font-size: 1.2rem;
  color: #94a3b8;
  margin-bottom: 2.5rem;
  line-height: 1.7;
  max-width: 600px;
}

.hero-stats {
  display: flex;
  gap: 3rem;
}

.stat-val {
  font-size: 1.5rem;
  font-weight: 700;
  color: #fff;
  margin-bottom: 0.25rem;
}

.stat-label {
  font-size: 0.85rem;
  color: #64748b;
  text-transform: uppercase;
  letter-spacing: 1px;
}

/* HERO VIZ */
.hero-viz {
  position: relative;
  width: 400px;
  height: 400px;
  display: flex;
  align-items: center;
  justify-content: center;
}

.plugin-matrix {
  width: 100%;
  height: 100%;
}

.core-node {
  animation: pulse-core 2s infinite;
}

.connection {
  opacity: 0;
  animation: pulse-connection 3s infinite;
}

.connection.c1 {
  animation-delay: 0s;
}
.connection.c2 {
  animation-delay: 0.75s;
}
.connection.c3 {
  animation-delay: 1.5s;
}
.connection.c4 {
  animation-delay: 2.25s;
}

/* PLUGIN TYPES */
.plugin-types-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
  gap: 1.5rem;
}

.type-card {
  background: rgba(15, 23, 42, 0.6);
  border-radius: 12px;
  padding: 2rem;
  display: flex;
  flex-direction: column;
  align-items: center;
  text-align: center;
  gap: 1rem;
  border: 2px solid;
}

.type-card.model-type {
  border-color: #3b82f6;
}

.type-card.sensor-type {
  border-color: #22c55e;
}

.type-card.world-type {
  border-color: #a855f7;
}

.type-card.visual-type {
  border-color: #ef4444;
}

.type-title {
  font-size: 1.2rem;
  font-weight: 700;
  color: #fff;
}

.type-desc {
  color: #cbd5e1;
  font-size: 0.9rem;
  line-height: 1.6;
}

.type-use {
  color: #94a3b8;
  font-size: 0.85rem;
  padding-top: 1rem;
  border-top: 1px solid rgba(148, 163, 184, 0.2);
  width: 100%;
}

/* LIFECYCLE */
.lifecycle-container {
  background: #0f172a;
  border: 1px solid #334155;
  border-radius: 12px;
  overflow: hidden;
}

.lifecycle-header {
  background: #1e293b;
  padding: 1rem;
  display: flex;
  align-items: center;
  gap: 0.75rem;
  border-bottom: 1px solid #334155;
  color: #fff;
  font-weight: 700;
}

.lifecycle-flow {
  padding: 2rem;
  display: flex;
  align-items: center;
  gap: 1rem;
  overflow-x: auto;
}

.lifecycle-stage {
  background: rgba(15, 23, 42, 0.6);
  border-radius: 12px;
  padding: 1.5rem;
  min-width: 200px;
  display: flex;
  flex-direction: column;
  gap: 0.75rem;
  border: 2px solid;
}

.lifecycle-stage.load {
  border-color: #3b82f6;
}

.lifecycle-stage.init {
  border-color: #22c55e;
}

.lifecycle-stage.update {
  border-color: #fbbf24;
}

.lifecycle-stage.fini {
  border-color: #ef4444;
}

.stage-number {
  width: 30px;
  height: 30px;
  border-radius: 50%;
  background: rgba(255, 255, 255, 0.1);
  display: flex;
  align-items: center;
  justify-content: center;
  font-weight: 700;
  color: #fff;
}

.stage-name {
  font-family: 'Fira Code', monospace;
  color: #fff;
  font-size: 1.1rem;
  font-weight: 700;
}

.stage-desc {
  color: #94a3b8;
  font-size: 0.85rem;
  line-height: 1.5;
}

.flow-arrow {
  font-size: 2rem;
  color: #64748b;
  flex-shrink: 0;
}

/* CODE SECTION */
.code-section {
  border: 1px solid #334155;
  border-radius: 12px;
  overflow: hidden;
}

.code-header {
  background: #1e293b;
  padding: 0.75rem 1rem;
  display: flex;
  align-items: center;
  gap: 0.75rem;
  border-bottom: 1px solid #334155;
  color: #94a3b8;
  font-family: monospace;
  font-size: 0.9rem;
}

/* API ACCESS */
.api-access-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
  gap: 1.5rem;
}

.api-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid #334155;
  border-radius: 12px;
  overflow: hidden;
}

.api-header {
  background: #1e293b;
  padding: 1rem;
  display: flex;
  align-items: center;
  gap: 0.75rem;
  border-bottom: 1px solid #334155;
  color: #fff;
  font-weight: 700;
}

.api-content {
  padding: 1.5rem;
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.api-content code {
  background: rgba(59, 130, 246, 0.1);
  padding: 0.5rem;
  border-radius: 6px;
  color: #60a5fa;
  font-family: 'Fira Code', monospace;
  font-size: 0.85rem;
}

.api-desc {
  color: #cbd5e1;
  font-size: 0.85rem;
  margin-top: -0.5rem;
}

/* ANIMATIONS */
@keyframes pulse-core {
  0%,
  100% {
    transform: scale(1);
    opacity: 1;
  }
  50% {
    transform: scale(1.1);
    opacity: 0.8;
  }
}

@keyframes pulse-connection {
  0% {
    opacity: 0;
  }
  50% {
    opacity: 1;
  }
  100% {
    opacity: 0;
  }
}

@media (max-width: 1024px) {
  .hero-section {
    flex-direction: column;
    padding: 3rem 1.5rem;
  }
  .lifecycle-flow {
    flex-direction: column;
  }
  .flow-arrow {
    transform: rotate(90deg);
  }
}

/* PID DIAGRAM */
.pid-viz-container {
  background: rgba(15, 23, 42, 0.6);
  padding: 2rem;
  border-radius: 16px;
  border: 1px solid rgba(148, 163, 184, 0.1);
}

.pid-diagram {
  display: flex;
  align-items: center;
  gap: 1rem;
  flex-wrap: wrap;
  justify-content: center;
}

.pid-block {
  background: #0f172a;
  border: 2px solid #3b82f6;
  border-radius: 8px;
  padding: 1rem;
  min-width: 120px;
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.5rem;
}

.block-label {
  font-size: 0.75rem;
  color: #94a3b8;
  text-transform: uppercase;
  letter-spacing: 0.5px;
}

.pid-block code {
  color: #60a5fa;
  font-family: 'Fira Code', monospace;
  font-size: 0.9rem;
}

.pid-arrow {
  font-size: 1.5rem;
  color: #64748b;
}

.pid-arrow.feedback {
  position: absolute;
  margin-top: 5rem;
}

/* CASE METRICS */
.case-metrics {
  display: flex;
  justify-content: center;
  gap: 2rem;
  flex-wrap: wrap;
}

.metric-card {
  background: rgba(15, 23, 42, 0.6);
  border: 2px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.75rem;
  min-width: 150px;
}

.metric-val {
  font-size: 1.5rem;
  font-weight: 700;
  color: #fff;
}

.metric-label {
  font-size: 0.8rem;
  color: #94a3b8;
  text-transform: uppercase;
  letter-spacing: 0.5px;
}

/* CHALLENGE CARD */
.challenge-card {
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid #ef4444;
  border-radius: 16px;
  padding: 2rem;
}

.challenge-header {
  display: flex;
  gap: 1.5rem;
  margin-bottom: 2rem;
}

.c-icon {
  font-size: 3rem;
  color: #ef4444;
}

.c-title {
  font-size: 1.25rem;
  font-weight: 700;
  color: #fca5a5;
  margin-bottom: 0.5rem;
}

.c-desc {
  color: #cbd5e1;
  font-size: 0.95rem;
}

.challenge-options {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.c-option {
  display: flex;
  gap: 1rem;
  padding: 1rem;
  background: rgba(0, 0, 0, 0.2);
  border-radius: 8px;
  cursor: default;
}

.c-option.correct {
  border: 1px solid #22c55e;
  background: rgba(34, 197, 94, 0.1);
}

.opt-radio {
  width: 30px;
  height: 30px;
  border-radius: 50%;
  border: 2px solid #64748b;
  display: flex;
  align-items: center;
  justify-content: center;
  font-weight: 700;
  color: #cbd5e1;
  flex-shrink: 0;
}

.c-option.correct .opt-radio {
  border-color: #22c55e;
  color: #22c55e;
}

.opt-text {
  color: #fff;
  flex: 1;
}

.opt-feedback {
  margin-top: 0.5rem;
  color: #86efac;
  font-size: 0.9rem;
}

/* VIDEO STYLES */
.video-container {
  background: #0f172a;
  border: 1px solid #334155;
  border-radius: 12px;
  overflow: hidden;
  padding: 1rem;
  box-shadow: 0 10px 30px -5px rgba(0, 0, 0, 0.5);
}

.video-wrapper {
  position: relative;
  padding-bottom: 56.25%;
  height: 0;
  overflow: hidden;
  border-radius: 8px;
  background: #000;
}

.video-wrapper iframe {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
}

.video-caption {
  padding: 1rem 0.5rem 0.5rem 0.5rem;
  color: #94a3b8;
  font-size: 0.9rem;
  display: flex;
  align-items: center;
  gap: 0.75rem;
}

/* SUMMARY */
.summary-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 1rem;
}

.summary-item {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 8px;
  padding: 1rem;
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.summary-item code {
  font-family: 'Fira Code', monospace;
  color: #fbbf24;
  font-size: 0.95rem;
  font-weight: 700;
}

.summary-item span {
  color: #cbd5e1;
  font-size: 0.85rem;
}
</style>

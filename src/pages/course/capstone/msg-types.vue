<template>
  <div class="course-content">
    <!-- INTRODUCCIÓN -->
    <div class="section-group">
      <TextBlock>
        Enviar texto plano (`"Hola 123"`) es ineficiente. El robot tiene que gastar tiempo
        procesando esa cadena para extraer el número.
        <br /><br />
        ROS 2 nos ofrece una biblioteca de <strong>Estructuras de Datos</strong> optimizadas. Son
        como formularios pre-impresos: sabes exactamente qué dato va en qué casilla.
      </TextBlock>
    </div>

    <!-- 1. LOS PAQUETES ESENCIALES -->
    <div class="section-group">
      <SectionTitle>1. La Santísima Trinidad de los Mensajes</SectionTitle>
      <TextBlock>
        El 90% del tiempo usarás mensajes de estos tres paquetes oficiales. No reinventes la rueda.
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-sm">
        <div class="col-12 col-md-4">
          <div class="pkg-card std">
            <div class="pkg-name">std_msgs</div>
            <div class="pkg-desc">Tipos primitivos básicos.</div>
            <ul class="msg-list">
              <li><code>String</code> (Texto)</li>
              <li><code>Int32</code> (Enteros)</li>
              <li><code>Float64</code> (Decimales)</li>
              <li><code>Bool</code> (Verdad/Falso)</li>
            </ul>
          </div>
        </div>
        <div class="col-12 col-md-4">
          <div class="pkg-card geometry">
            <div class="pkg-name">geometry_msgs</div>
            <div class="pkg-desc">Física y Movimiento.</div>
            <ul class="msg-list">
              <li><code>Twist</code> (Velocidad Lin/Ang)</li>
              <li><code>Pose</code> (Posición + Orientación)</li>
              <li><code>Point</code> (X, Y, Z)</li>
            </ul>
          </div>
        </div>
        <div class="col-12 col-md-4">
          <div class="pkg-card sensor">
            <div class="pkg-name">sensor_msgs</div>
            <div class="pkg-desc">Datos del mundo real.</div>
            <ul class="msg-list">
              <li><code>LaserScan</code> (LIDAR)</li>
              <li><code>Image</code> (Cámara)</li>
              <li><code>Imu</code> (Acelerómetro)</li>
            </ul>
          </div>
        </div>
      </div>
    </div>

    <!-- 2. ANATOMÍA DE UN MENSAJE -->
    <div class="section-group">
      <SectionTitle>2. Diseccionando un Mensaje (Twist)</SectionTitle>
      <TextBlock>
        El mensaje más famoso es <code>geometry_msgs/msg/Twist</code>. Es el que usaste para mover
        la tortuga. ¿Qué tiene por dentro?
      </TextBlock>

      <SplitBlock>
        <template #left>
          <CodeBlock
            title="Estructura (Twist.msg)"
            lang="yaml"
            content="# Velocidad Lineal (m/s)
Vector3  linear
    float64 x
    float64 y
    float64 z

# Velocidad Angular (rad/s)
Vector3  angular
    float64 x
    float64 y
    float64 z"
          />
        </template>
        <template #right>
          <div class="usage-example q-pa-md">
            <div class="text-subtitle2 text-primary q-mb-sm">Cómo usarlo en Python:</div>
            <CodeBlock
              lang="python"
              content="from geometry_msgs.msg import Twist

cmd = Twist()
# Mover hacia adelante
cmd.linear.x = 2.0
# Girar a la izquierda
cmd.angular.z = 1.5

publisher.publish(cmd)"
            />
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 3. MENSAJES PERSONALIZADOS -->
    <div class="section-group">
      <SectionTitle>3. Creando tus Propios Mensajes</SectionTitle>
      <TextBlock>
        A veces los estándares no bastan. Imagina que quieres enviar los datos de un "Escáner
        Alienígena". Necesitas crear un archivo <code>.msg</code>.
      </TextBlock>

      <AlertBlock type="warning" title="Regla de Oro">
        Los mensajes personalizados deben vivir en su propio paquete (ej:
        <code>my_robot_interfaces</code>), separados del código Python. Esto evita problemas de
        compilación cíclica.
      </AlertBlock>

      <div class="q-my-md">
        <CodeBlock
          title="AlienScan.msg"
          lang="yaml"
          content="int64 id_alien
string especie
float32 nivel_peligro
bool es_hostil"
        />
      </div>

      <StepsBlock
        :steps="[
          'Crear paquete CMake (no Python): ros2 pkg create --build-type ament_cmake my_interfaces',
          'Crear carpeta msg/ y el archivo AlienScan.msg dentro.',
          'Editar CMakeLists.txt para añadir: rosidl_generate_interfaces(...).',
          'Editar package.xml para añadir dependencias de rosidl.',
          'Compilar.',
        ]"
      />
    </div>

    <!-- 4. CLI DE INTERFACES -->
    <div class="section-group">
      <SectionTitle>4. El Explorador (ros2 interface)</SectionTitle>
      <TextBlock> Nunca memorices las estructuras. Usa la terminal para consultarlas. </TextBlock>

      <CodeBlock
        title="Consultas Rápidas"
        lang="bash"
        content="# Listar todos los mensajes disponibles
ros2 interface list

# Ver qué tiene adentro el mensaje LaserScan
ros2 interface show sensor_msgs/msg/LaserScan

# Ver qué tiene adentro un String
ros2 interface show std_msgs/msg/String"
      />
    </div>

    <!-- RETO -->
    <div class="section-group">
      <SectionTitle>🏆 Reto de Telemetría</SectionTitle>
      <TextBlock>
        1. Tu nave necesita reportar su nivel de combustible y coordenadas. 2. No crees un mensaje
        nuevo. Busca en la lista oficial. 3. ¿Qué mensaje de <code>geometry_msgs</code> sirve para
        posición (X,Y,Z)? (R: Point o Pose). 4. ¿Qué mensaje de <code>std_msgs</code> sirve para el
        combustible (0.0 a 100.0)? (R: Float32). <br /><br />
        <strong>Misión:</strong> Modifica tu nodo 'Radar' para que publique un
        <code>geometry_msgs/Point</code>
        con coordenadas aleatorias de donde viene la amenaza.
      </TextBlock>
    </div>
  </div>
</template>

<script setup lang="ts">
import SectionTitle from 'components/content/SectionTitle.vue';
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SplitBlock from 'components/content/SplitBlock.vue';
import StepsBlock from 'components/content/StepsBlock.vue';
</script>

<style scoped>
.section-group {
  margin-bottom: 3rem;
}

.pkg-card {
  background: var(--bg-surface-solid);
  border-radius: 12px;
  padding: 1.2rem;
  height: 100%;
  border: 1px solid rgba(148, 163, 184, 0.2);
  transition: transform 0.2s;
}
.pkg-card:hover {
  transform: translateY(-5px);
}

.pkg-card.std {
  border-top: 4px solid #94a3b8;
}
.pkg-card.geometry {
  border-top: 4px solid #f59e0b;
}
.pkg-card.sensor {
  border-top: 4px solid #ef4444;
}

.pkg-name {
  font-family: 'Fira Code', monospace;
  font-weight: bold;
  font-size: 1.1rem;
  color: var(--text-primary);
  margin-bottom: 5px;
}
.pkg-desc {
  font-size: 0.85rem;
  color: var(--text-secondary);
  margin-bottom: 10px;
}
.msg-list {
  padding-left: 1.2rem;
  color: var(--text-muted);
  font-size: 0.9rem;
}
.msg-list code {
  color: #a5f3fc;
}

.usage-example {
  background: var(--bg-surface-hover);
  border-radius: 8px;
  height: 100%;
}
</style>

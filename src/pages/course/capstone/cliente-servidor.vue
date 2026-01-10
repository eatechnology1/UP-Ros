<template>
  <div class="course-content">
    <!-- INTRODUCCIÓN -->
    <div class="section-group">
      <div class="row items-center q-gutter-md">
        <div class="col-grow">
          <TextBlock>
            Los Tópicos son geniales para datos continuos (sensores), pero pésimos para **acciones
            puntuales**.
            <br /><br />
            Imagina que quieres tomar una foto. Si usas un tópico, el robot tomaría 30 fotos por
            segundo infinitamente. Para acciones como "Tomar foto", "Guardar mapa" o "Reiniciar
            sistema", usamos el paradigma
            <strong>Cliente-Servidor (Servicios)</strong>.
          </TextBlock>
        </div>
        <div class="col-auto">
          <div class="text-h1">🤵</div>
        </div>
      </div>
    </div>

    <!-- 1. DIFERENCIA CONCEPTUAL -->
    <div class="section-group">
      <SectionTitle>1. Streaming vs. Transacción</SectionTitle>

      <SplitBlock>
        <template #left>
          <div class="concept-card topic">
            <div class="header">📺 TÓPICO (Pub/Sub)</div>
            <div class="body">
              <p><strong>Relación:</strong> Muchos a Muchos.</p>
              <p><strong>Analogía:</strong> La Radio.</p>
              <p><strong>Comportamiento:</strong> "Yo hablo y no me importa si me escuchas".</p>
              <div class="arrow-anim streaming">>>> >>> >>></div>
            </div>
          </div>
        </template>
        <template #right>
          <div class="concept-card service">
            <div class="header">🤝 SERVICIO (Cli/Srv)</div>
            <div class="body">
              <p><strong>Relación:</strong> Uno a Uno (puntual).</p>
              <p><strong>Analogía:</strong> Una llamada telefónica.</p>
              <p><strong>Comportamiento:</strong> "Te pido algo, espero, y tú me respondes".</p>
              <div class="interaction-anim">
                <span class="req">Petición ➡</span><br />
                <span class="res">⬅ Respuesta</span>
              </div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 2. ANATOMÍA DEL SERVICIO (.SRV) -->
    <div class="section-group">
      <SectionTitle>2. La Estructura .srv</SectionTitle>
      <TextBlock>
        A diferencia de los mensajes (`.msg`), los servicios (`.srv`) tienen DOS partes separadas
        por tres guiones `---`.
      </TextBlock>

      <div class="srv-visual q-my-md">
        <div class="srv-box request">
          <div class="label">REQUEST (Lo que pides)</div>
          <code>string tipo_cafe</code><br />
          <code>bool con_azucar</code>
        </div>

        <div class="srv-separator">---</div>

        <div class="srv-box response">
          <div class="label">RESPONSE (Lo que recibes)</div>
          <code>bool exito</code><br />
          <code>string mensaje_mayordomo</code>
        </div>
      </div>
    </div>

    <!-- 3. EJEMPLO REAL EN CÓDIGO -->
    <div class="section-group">
      <SectionTitle>3. El Nodo Mayordomo (Servidor)</SectionTitle>
      <TextBlock>
        Vamos a crear un nodo que ofrezca el servicio de sumar dos números (el "Hola Mundo" de los
        servicios).
        <br />
        Usaremos el tipo predefinido <code>example_interfaces/srv/AddTwoInts</code>.
      </TextBlock>

      <CodeBlock
        title="servidor_suma.py"
        lang="python"
        content="import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class SumaService(Node):
    def __init__(self):
        super().__init__('suma_service_node')

        # 1. Crear el Servidor
        # (Tipo, 'nombre_servicio', callback)
        self.srv = self.create_service(
            AddTwoInts,
            'sumar_dos_ints',
            self.sumar_callback
        )
        self.get_logger().info('🔢 Servicio de Suma LISTO. Esperando clientes...')

    # 2. El Callback recibe Request y Response
    def sumar_callback(self, request, response):
        # request.a y request.b vienen del cliente
        resultado = request.a + request.b

        # Llenamos la respuesta
        response.sum = resultado

        self.get_logger().info(f'Me pidieron sumar {request.a} + {request.b}. Respondí {response.sum}')

        # IMPORTANTE: Devolver el objeto response
        return response

def main():
    rclpy.init()
    rclpy.spin(SumaService())
    rclpy.shutdown()"
        :copyable="true"
      />
    </div>

    <!-- 4. LLAMANDO AL SERVICIO (CLIENTE) -->
    <div class="section-group">
      <SectionTitle>4. Invocando al Mayordomo</SectionTitle>
      <TextBlock>
        Podemos llamar al servicio desde código (Python) o, más fácil, desde la terminal para
        probarlo.
      </TextBlock>

      <SplitBlock>
        <template #left>
          <div class="terminal-window">
            <div class="header">Opción A: Terminal (Rápido)</div>
            <CodeBlock
              lang="bash"
              content="# Estructura: ros2 service call [nombre] [tipo] [datos]
ros2 service call /sumar_dos_ints example_interfaces/srv/AddTwoInts '{a: 5, b: 10}'"
            />
            <div class="output-text q-mt-sm">
              requester: making request: example_interfaces.srv.AddTwoInts_Request(a=5, b=10)<br />
              response:<br />
              example_interfaces.srv.AddTwoInts_Response(sum=15)
            </div>
          </div>
        </template>
        <template #right>
          <div class="terminal-window">
            <div class="header">Opción B: Python (Asíncrono)</div>
            <TextBlock>
              Escribir un cliente en Python es complejo porque **la llamada puede tardar**.
              <br />
              Si usas `client.call()`, bloqueas el robot.
              <br />
              Debes usar `client.call_async()` y esperar el **Futuro** (como una Promesa en JS).
            </TextBlock>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 5. SYNCHRONOUS VS ASYNCHRONOUS -->
    <div class="section-group">
      <SectionTitle>5. El Peligro del Bloqueo (Otra vez)</SectionTitle>

      <AlertBlock type="danger" title="⚠️ Deadlock">
        Nunca llames a un servicio SÍNCRONO desde dentro de un Callback.
        <br /><br />
        El nodo se congela esperando la respuesta, pero como está congelado, no puede procesar la
        respuesta cuando llega. Se quedan mirándose eternamente (Deadlock).
        <br />
        <strong>Siempre usa llamadas ASÍNCRONAS.</strong>
      </AlertBlock>
    </div>

    <!-- RETO -->
    <div class="section-group">
      <SectionTitle>🏆 Reto del Mayordomo</SectionTitle>
      <TextBlock>
        1. Ejecuta el servidor de suma. 2. Intenta llamarlo desde la terminal con números negativos.
        ¿Funciona? 3. <strong>Reto de Código:</strong> Modifica el servidor para que solo sume
        números positivos. Si recibe un negativo, devuelve `-1` y imprime un error "¡Solo sumo cosas
        positivas!".
        <br />
        (Esto te enseña a validar datos dentro del servicio).
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
</script>

<style scoped>
.section-group {
  margin-bottom: 3rem;
}

/* Concept Cards */
.concept-card {
  border-radius: 12px;
  padding: 1.5rem;
  height: 100%;
  color: white;
  display: flex;
  flex-direction: column;
}
.concept-card.topic {
  background: linear-gradient(145deg, rgba(59, 130, 246, 0.2), rgba(15, 23, 42, 0.9));
  border: 1px solid #3b82f6;
}
.concept-card.service {
  background: linear-gradient(145deg, rgba(16, 185, 129, 0.2), rgba(15, 23, 42, 0.9));
  border: 1px solid #10b981;
}

.header {
  font-weight: 800;
  font-size: 1.2rem;
  margin-bottom: 1rem;
  text-align: center;
  letter-spacing: 1px;
}
.body p {
  font-size: 0.9rem;
  margin-bottom: 8px;
  color: #e2e8f0;
}

/* Animaciones */
.arrow-anim.streaming {
  color: #60a5fa;
  font-weight: bold;
  font-family: monospace;
  margin-top: 15px;
  animation: slideRight 1s infinite linear;
}
@keyframes slideRight {
  0% {
    transform: translateX(0);
    opacity: 0;
  }
  50% {
    opacity: 1;
  }
  100% {
    transform: translateX(20px);
    opacity: 0;
  }
}

.interaction-anim {
  margin-top: 15px;
  font-family: monospace;
  font-weight: bold;
  text-align: center;
}
.interaction-anim .req {
  color: #fcd34d;
  display: block;
  margin-bottom: 5px;
}
.interaction-anim .res {
  color: #34d399;
  display: block;
}

/* Visualización .SRV */
.srv-visual {
  background: #1e293b;
  border: 1px solid #475569;
  border-radius: 8px;
  padding: 20px;
  font-family: 'Fira Code', monospace;
  text-align: center;
  max-width: 400px;
  margin: 0 auto;
}
.srv-box {
  padding: 10px;
  border-radius: 6px;
}
.srv-box.request {
  background: rgba(251, 191, 36, 0.1);
  border: 1px dashed #fbbf24;
}
.srv-box.response {
  background: rgba(52, 211, 153, 0.1);
  border: 1px dashed #34d399;
}
.srv-separator {
  color: #94a3b8;
  font-weight: bold;
  letter-spacing: 5px;
  margin: 10px 0;
}
.label {
  font-size: 0.7rem;
  color: #94a3b8;
  margin-bottom: 5px;
  text-transform: uppercase;
}

.terminal-window {
  background: black;
  border: 1px solid #333;
  padding: 15px;
  border-radius: 8px;
}
.output-text {
  color: #ccc;
  font-size: 0.8rem;
  font-family: 'Fira Code', monospace;
}
</style>

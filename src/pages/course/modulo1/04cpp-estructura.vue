<template>
  <LessonContainer>
    <!-- INTRO -->
    <TextBlock>
      C++ divide el código en dos tipos de archivos: <strong>headers (.hpp)</strong> que declaran
      QUÉ existe, y <strong>source (.cpp)</strong> que implementa CÓMO funciona. Esta separación
      permite compilación modular y reutilización de código. <br /><br />
      Esta lección te enseña la estructura de proyectos C++ en ROS 2, smart pointers, namespaces, y
      las mejores prácticas de organización de código.
    </TextBlock>

    <!-- .HPP VS .CPP -->
    <div class="section-group">
      <SectionTitle>1. La Gran División: .hpp vs .cpp</SectionTitle>

      <div class="file-comparison">
        <div class="file-card header">
          <div class="file-header">
            <q-icon name="menu_book" size="3rem" color="orange-4" />
            <div class="file-title">.hpp (Header)</div>
            <div class="file-subtitle">El "Menú" - Declaración</div>
          </div>
          <div class="file-content">
            <div class="file-desc">
              Le dice al compilador <strong>QUÉ</strong> existe. Lista las funciones y clases, pero
              no su implementación. Es el contrato público.
            </div>
            <CodeBlock
              title="robot.hpp"
              lang="cpp"
              content="#ifndef ROBOT_HPP
#define ROBOT_HPP

class Robot {
public:
  Robot();  // Constructor
  void mover(double velocidad);
  int getBateria();

private:
  int bateria_;
  double posicion_x_;
};

#endif  // ROBOT_HPP"
              :copyable="true"
            />
          </div>
        </div>

        <div class="file-card source">
          <div class="file-header">
            <q-icon name="code" size="3rem" color="blue-4" />
            <div class="file-title">.cpp (Source)</div>
            <div class="file-subtitle">La "Cocina" - Implementación</div>
          </div>
          <div class="file-content">
            <div class="file-desc">
              Le dice al compilador <strong>CÓMO</strong> funciona. Aquí está la lógica real, las
              matemáticas y el código.
            </div>
            <CodeBlock
              title="robot.cpp"
              lang="cpp"
              content='#include "robot.hpp"

Robot::Robot() : bateria_(100), posicion_x_(0.0) {
  // Constructor
}

void Robot::mover(double velocidad) {
  posicion_x_ += velocidad;
  bateria_ -= 5;
}

int Robot::getBateria() {
  return bateria_;
}'
              :copyable="true"
            />
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <AlertBlock type="info" title="¿Por qué separar?">
          <strong>Compilación más rápida:</strong> Si cambias la implementación (.cpp), solo
          recompilas ese archivo. Si cambias el header (.hpp), recompilas TODO lo que lo incluye.
          <br /><br />
          <strong>Encapsulación:</strong> Otros desarrolladores solo ven el "menú" (API pública), no
          la "cocina" (detalles internos).
        </AlertBlock>
      </div>
    </div>

    <!-- SCOPE OPERATOR -->
    <div class="section-group">
      <SectionTitle>2. El Operador :: (Scope Resolution)</SectionTitle>
      <TextBlock>
        Cuando defines una función fuera de la clase (en el .cpp), debes usar
        <code>::</code> para indicar a qué clase pertenece.
      </TextBlock>

      <div class="scope-demo q-mt-md">
        <div class="scope-wrong">
          <div class="scope-label">❌ Incorrecto</div>
          <CodeBlock
            lang="cpp"
            content="// robot.cpp
void mover() {  // ¿Quién se mueve?
  // El compilador no sabe que esto es del Robot
}"
          />
        </div>

        <div class="scope-arrow">
          <q-icon name="arrow_forward" size="2rem" color="yellow-6" />
        </div>

        <div class="scope-correct">
          <div class="scope-label">✅ Correcto</div>
          <CodeBlock
            lang="cpp"
            content="// robot.cpp
void Robot::mover() {  // Pertenece a Robot
  // Ahora el compilador sabe
}"
          />
        </div>
      </div>

      <div class="q-mt-md">
        <SectionTitle>Otros Usos de ::</SectionTitle>
        <div class="row q-col-gutter-md">
          <div class="col-12 col-md-6">
            <div class="scope-use-card">
              <div class="use-title">Acceder a Namespace</div>
              <CodeBlock lang="cpp" content="std::cout << 'Hola';" :copyable="true" />
            </div>
          </div>
          <div class="col-12 col-md-6">
            <div class="scope-use-card">
              <div class="use-title">Acceder a Miembro Estático</div>
              <CodeBlock lang="cpp" content="Robot::MAX_VELOCIDAD" :copyable="true" />
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- SMART POINTERS -->
    <div class="section-group">
      <SectionTitle>3. Smart Pointers: Memoria Moderna</SectionTitle>
      <AlertBlock type="warning" title="💀 Olvida new y delete">
        En C++ moderno (y ROS 2), <strong>NUNCA</strong> uses <code>new</code> y
        <code>delete</code> manualmente. Los smart pointers gestionan la memoria automáticamente.
      </AlertBlock>

      <div class="smart-pointers q-mt-md">
        <div class="pointer-card shared">
          <div class="pointer-header">
            <q-icon name="share" size="2rem" />
            <span>std::shared_ptr</span>
          </div>
          <div class="pointer-content">
            <div class="pointer-desc">
              <strong>Propiedad compartida.</strong> Varios objetos pueden "sostener" el mismo
              recurso. Se destruye cuando el último dueño desaparece.
            </div>
            <CodeBlock
              lang="cpp"
              content="auto nodo = std::make_shared<MiNodo>();
// Cuenta de referencias: 1

auto copia = nodo;
// Cuenta de referencias: 2

// Cuando ambos salen de scope, se destruye automáticamente"
              :copyable="true"
            />
            <div class="pointer-use">
              <strong>Uso en ROS 2:</strong> Nodos, publishers, subscriptions
            </div>
          </div>
        </div>

        <div class="pointer-card unique">
          <div class="pointer-header">
            <q-icon name="lock" size="2rem" />
            <span>std::unique_ptr</span>
          </div>
          <div class="pointer-content">
            <div class="pointer-desc">
              <strong>Propiedad exclusiva.</strong> Solo un objeto puede poseerlo. No se puede
              copiar, solo mover.
            </div>
            <CodeBlock
              lang="cpp"
              content="auto sensor = std::make_unique<Lidar>();
// Solo este objeto posee el sensor

// auto copia = sensor;  // ❌ Error de compilación
auto movido = std::move(sensor);  // ✅ Transferencia de propiedad"
              :copyable="true"
            />
            <div class="pointer-use">
              <strong>Uso en ROS 2:</strong> Drivers de hardware, recursos exclusivos
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- ESTRUCTURA DE PROYECTO -->
    <div class="section-group">
      <SectionTitle>4. Estructura de Proyecto C++ en ROS 2</SectionTitle>

      <div class="project-tree q-mt-md">
        <div class="tree-title">mi_robot_cpp/</div>
        <div class="tree-content">
          <div class="tree-item file">
            <q-icon name="description" color="grey-5" />
            <span>CMakeLists.txt</span>
            <div class="item-note">Receta de compilación</div>
          </div>
          <div class="tree-item file">
            <q-icon name="description" color="grey-5" />
            <span>package.xml</span>
            <div class="item-note">Metadatos del paquete</div>
          </div>

          <div class="tree-item folder highlight">
            <q-icon name="folder" color="orange-4" />
            <span>include/mi_robot_cpp/</span>
            <div class="item-note">Headers (.hpp)</div>
          </div>
          <div class="tree-children">
            <div class="tree-item file">
              <q-icon name="description" color="orange-3" />
              <span>control_node.hpp</span>
            </div>
            <div class="tree-item file">
              <q-icon name="description" color="orange-3" />
              <span>motor_driver.hpp</span>
            </div>
          </div>

          <div class="tree-item folder highlight">
            <q-icon name="folder" color="blue-4" />
            <span>src/</span>
            <div class="item-note">Implementación (.cpp)</div>
          </div>
          <div class="tree-children">
            <div class="tree-item file">
              <q-icon name="description" color="blue-3" />
              <span>control_node.cpp</span>
            </div>
            <div class="tree-item file">
              <q-icon name="description" color="blue-3" />
              <span>motor_driver.cpp</span>
            </div>
            <div class="tree-item file">
              <q-icon name="description" color="green-4" />
              <span>main.cpp</span>
              <div class="item-note">Punto de entrada</div>
            </div>
          </div>

          <div class="tree-item folder">
            <q-icon name="folder" color="purple-4" />
            <span>launch/</span>
          </div>
          <div class="tree-item folder">
            <q-icon name="folder" color="cyan-4" />
            <span>config/</span>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <AlertBlock type="info" title="Regla de Oro">
          Los headers (.hpp) van en <code>include/nombre_paquete/</code>
          <br />
          La implementación (.cpp) va en <code>src/</code>
        </AlertBlock>
      </div>
    </div>

    <!-- INCLUDES -->
    <div class="section-group">
      <SectionTitle>5. Sistema de Includes</SectionTitle>

      <div class="row q-col-gutter-md">
        <div class="col-12 col-md-6">
          <div class="include-card system">
            <div class="include-header">
              <q-icon name="library_books" size="md" />
              <span>Librerías del Sistema</span>
            </div>
            <CodeBlock
              lang="cpp"
              content="#include <iostream>
#include <vector>
#include <rclcpp/rclcpp.hpp>"
            />
            <div class="include-note">
              Usa <code>&lt; &gt;</code> para librerías estándar y de ROS 2
            </div>
          </div>
        </div>

        <div class="col-12 col-md-6">
          <div class="include-card local">
            <div class="include-header">
              <q-icon name="folder" size="md" />
              <span>Headers Locales</span>
            </div>
            <CodeBlock
              lang="cpp"
              content='#include "mi_robot_cpp/control_node.hpp"
#include "motor_driver.hpp"'
            />
            <div class="include-note">Usa <code>" "</code> para tus propios headers</div>
          </div>
        </div>
      </div>
    </div>

    <!-- ERRORES COMUNES -->
    <div class="section-group">
      <SectionTitle>Errores Comunes</SectionTitle>

      <q-expansion-item
        icon="error"
        label="undefined reference to `Robot::mover()'"
        header-class="error-header"
      >
        <div class="error-content">
          <strong>Causa:</strong> Declaraste la función en .hpp pero no la implementaste en .cpp, o
          olvidaste agregar el .cpp a CMakeLists.txt <br /><br />
          <strong>Solución:</strong>
          <CodeBlock
            lang="cmake"
            content="add_executable(mi_nodo
  src/main.cpp
  src/robot.cpp  # ← Asegúrate de incluirlo
)"
            :copyable="true"
          />
        </div>
      </q-expansion-item>

      <q-expansion-item
        icon="error"
        label="'Robot' does not name a type"
        header-class="error-header"
        class="q-mt-sm"
      >
        <div class="error-content">
          <strong>Causa:</strong> Olvidaste incluir el header o hay un include circular <br /><br />
          <strong>Solución:</strong> Agrega <code>#include "robot.hpp"</code> al inicio del archivo
        </div>
      </q-expansion-item>
    </div>

    <!-- VIDEO -->
    <div class="section-group">
      <SectionTitle>📹 Video Complementario</SectionTitle>
      <div class="video-container">
        <div class="video-wrapper">
          <iframe
            src="https://www.youtube.com/embed/Romc22GgusU"
            title="Estructura de Proyectos C++ en ROS 2"
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
          <q-icon name="info" color="blue-4" size="sm" />
          Video En progreso
        </div>
      </div>
    </div>

    <!-- RESUMEN -->
    <div class="section-group q-mb-xl">
      <SectionTitle>📝 Resumen</SectionTitle>
      <div class="summary-grid">
        <div class="summary-item">
          <code>.hpp</code>
          <span>Declaraciones (QUÉ existe)</span>
        </div>
        <div class="summary-item">
          <code>.cpp</code>
          <span>Implementación (CÓMO funciona)</span>
        </div>
        <div class="summary-item">
          <code>::</code>
          <span>Operador de scope (pertenece a)</span>
        </div>
        <div class="summary-item">
          <code>std::shared_ptr</code>
          <span>Propiedad compartida</span>
        </div>
        <div class="summary-item">
          <code>std::unique_ptr</code>
          <span>Propiedad exclusiva</span>
        </div>
        <div class="summary-item">
          <code>include/</code>
          <span>Carpeta para headers</span>
        </div>
      </div>
    </div>
  </LessonContainer>
</template>

<script setup lang="ts">
import LessonContainer from 'components/content/LessonContainer.vue';
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
</script>

<style scoped>
.section-group {
  margin-bottom: 3.5rem;
}

.file-comparison {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(350px, 1fr));
  gap: 2rem;
  margin-top: 1.5rem;
}

.file-card {
  background: var(--bg-surface);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  overflow: hidden;
}

.file-card.header {
  border-top: 4px solid #f97316;
}

.file-card.source {
  border-top: 4px solid #3b82f6;
}

.file-header {
  padding: 2rem;
  text-align: center;
  background: var(--bg-surface-hover);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
}

.file-title {
  font-size: 1.5rem;
  font-weight: 700;
  color: var(--text-primary);
  margin-top: 1rem;
}

.file-subtitle {
  font-size: 0.9rem;
  color: var(--text-muted);
  margin-top: 0.5rem;
}

.file-content {
  padding: 1.5rem;
}

.file-desc {
  color: var(--text-secondary);
  margin-bottom: 1.5rem;
  line-height: 1.6;
}

.scope-demo {
  display: grid;
  grid-template-columns: 1fr auto 1fr;
  gap: 2rem;
  align-items: center;
  padding: 2rem;
  background: var(--bg-surface);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
}

.scope-label {
  font-size: 1.1rem;
  font-weight: 700;
  margin-bottom: 1rem;
  text-align: center;
}

.scope-arrow {
  color: var(--text-warning, #d97706);
}

.scope-use-card {
  background: var(--bg-surface);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  height: 100%;
}

.use-title {
  font-size: 1.05rem;
  font-weight: 700;
  color: var(--text-primary);
  margin-bottom: 1rem;
}

.smart-pointers {
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.pointer-card {
  background: var(--bg-surface);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
}

.pointer-card.shared {
  border-top: 4px solid #3b82f6;
}

.pointer-card.unique {
  border-top: 4px solid #a855f7;
}

.pointer-header {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 1rem 1.5rem;
  background: var(--bg-surface-hover);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  font-weight: 700;
  font-size: 1.1rem;
  color: var(--text-primary);
}

.pointer-content {
  padding: 1.5rem;
}

.pointer-desc {
  color: var(--text-secondary);
  margin-bottom: 1.5rem;
  line-height: 1.6;
}

.pointer-use {
  margin-top: 1rem;
  padding: 0.75rem;
  background: rgba(59, 130, 246, 0.1);
  border-radius: 6px;
  font-size: 0.85rem;
  color: #93c5fd;
}

.project-tree {
  background: var(--bg-surface);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
  font-family: 'Fira Code', monospace;
}

.tree-title {
  font-size: 1.25rem;
  font-weight: 700;
  color: var(--text-info, #2563eb);
  margin-bottom: 1.5rem;
}

.tree-content {
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.tree-item {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  padding: 0.75rem;
  border-radius: 6px;
  transition: background 0.2s;
}

.tree-item:hover {
  background: var(--bg-surface-hover);
}

.tree-item.folder.highlight {
  background: rgba(59, 130, 246, 0.1);
  border-left: 3px solid #3b82f6;
}

.tree-item span {
  color: var(--text-primary);
  font-weight: 500;
}

.item-note {
  margin-left: auto;
  font-size: 0.75rem;
  color: var(--text-muted);
  font-family: sans-serif;
}

.tree-children {
  margin-left: 2rem;
  border-left: 1px solid rgba(148, 163, 184, 0.2);
  padding-left: 1rem;
}

.include-card {
  background: var(--bg-surface);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
  height: 100%;
}

.include-header {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 1rem 1.5rem;
  background: var(--bg-surface-hover);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  font-weight: 700;
  color: var(--text-primary);
}

.include-note {
  margin-top: 1rem;
  padding: 0.75rem;
  background: rgba(59, 130, 246, 0.1);
  border-radius: 6px;
  font-size: 0.85rem;
  color: var(--text-muted);
}

:deep(.error-header) {
  background: rgba(239, 68, 68, 0.1);
  border: 1px solid rgba(239, 68, 68, 0.3);
  border-radius: 8px;
  color: var(--text-danger, #dc2626);
}

.error-content {
  background: var(--bg-surface);
  padding: 1.5rem;
  color: var(--text-secondary);
}

.video-container {
  background: var(--bg-surface);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 1.5rem;
}

.video-wrapper {
  position: relative;
  padding-bottom: 56.25%;
  height: 0;
  overflow: hidden;
  border-radius: 12px;
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
  display: flex;
  align-items: center;
  gap: 8px;
  margin-top: 1rem;
  padding: 0.75rem;
  background: rgba(59, 130, 246, 0.1);
  border-radius: 8px;
  color: var(--text-muted);
  font-size: 0.85rem;
}

.summary-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
  gap: 1rem;
}

.summary-item {
  background: var(--bg-surface);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 8px;
  padding: 1rem;
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.summary-item code {
  font-family: 'Fira Code', monospace;
  color: var(--text-code);
  font-size: 1rem;
}

.summary-item span {
  color: var(--text-secondary);
  font-size: 0.85rem;
}

@media (max-width: 768px) {
  .scope-demo {
    grid-template-columns: 1fr;
  }

  .scope-arrow {
    transform: rotate(90deg);
  }
}
</style>

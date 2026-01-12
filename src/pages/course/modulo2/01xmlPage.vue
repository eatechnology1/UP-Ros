<template>
  <LessonContainer>
    <!-- INTRO -->
    <TextBlock>
      XML (eXtensible Markup Language) es el lenguaje de configuración de ROS 2. No es código
      ejecutable, es <strong>datos estructurados</strong> que describen cómo debe comportarse tu
      robot. <br /><br />
      En ROS 2, usarás XML para definir paquetes (<code>package.xml</code>), robots físicos
      (<code>URDF</code>), y lanzar múltiples nodos (<code>launch files</code>).
    </TextBlock>

    <AlertBlock type="info" title="¿Por qué XML y no JSON o YAML?">
      <strong>XML:</strong> Estándar de la industria robótica, validación estricta, herramientas
      maduras
      <br />
      <strong>YAML:</strong> ROS 2 también lo soporta para configuración (más legible)
      <br />
      <strong>JSON:</strong> Usado en APIs web, no común en robótica <br /><br />
      XML es <strong>verboso</strong> pero <strong>explícito</strong>. En robótica, la claridad vale
      más que la brevedad.
    </AlertBlock>

    <!-- ANATOMÍA -->
    <div class="section-group">
      <SectionTitle>1. Anatomía de una Etiqueta XML</SectionTitle>

      <div class="anatomy-demo">
        <div class="anatomy-visual">
          <div class="tag-breakdown">
            <span class="bracket">&lt;</span>
            <span class="element">link</span>
            <span class="attribute">name</span>
            <span class="equals">=</span>
            <span class="value">"base_link"</span>
            <span class="bracket">&gt;</span>
            <span class="content">Contenido aquí</span>
            <span class="bracket">&lt;/</span>
            <span class="element">link</span>
            <span class="bracket">&gt;</span>
          </div>

          <div class="annotations">
            <div class="annotation element-ann">
              <q-icon name="arrow_upward" />
              <span>Elemento</span>
            </div>
            <div class="annotation attribute-ann">
              <q-icon name="arrow_upward" />
              <span>Atributo</span>
            </div>
            <div class="annotation value-ann">
              <q-icon name="arrow_upward" />
              <span>Valor</span>
            </div>
            <div class="annotation content-ann">
              <q-icon name="arrow_upward" />
              <span>Contenido</span>
            </div>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <SectionTitle>Atributos vs Contenido: ¿Cuándo usar cada uno?</SectionTitle>
        <div class="row q-col-gutter-md q-mt-sm">
          <div class="col-12 col-md-6">
            <div class="comparison-card attributes">
              <div class="comparison-header">
                <q-icon name="settings" size="md" />
                <span>Atributos (Propiedades)</span>
              </div>
              <div class="comparison-content">
                <CodeBlock
                  lang="xml"
                  content='<cylinder radius="0.5" length="1.0" />'
                  :copyable="true"
                />
                <div class="comparison-note">
                  <strong>Usar para:</strong>
                  <ul>
                    <li>Valores numéricos (radio, longitud)</li>
                    <li>IDs y nombres</li>
                    <li>Flags booleanos</li>
                    <li>Metadatos cortos</li>
                  </ul>
                </div>
              </div>
            </div>
          </div>

          <div class="col-12 col-md-6">
            <div class="comparison-card content">
              <div class="comparison-header">
                <q-icon name="text_fields" size="md" />
                <span>Contenido (Texto/Hijos)</span>
              </div>
              <div class="comparison-content">
                <CodeBlock
                  lang="xml"
                  content="<description>
  Robot móvil con 4 ruedas
</description>"
                  :copyable="true"
                />
                <div class="comparison-note">
                  <strong>Usar para:</strong>
                  <ul>
                    <li>Texto largo (descripciones)</li>
                    <li>Etiquetas anidadas</li>
                    <li>Listas de elementos</li>
                    <li>Estructuras complejas</li>
                  </ul>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- REGLAS DE ORO -->
    <div class="section-group">
      <SectionTitle>2. Las 5 Reglas de Oro del XML</SectionTitle>

      <div class="rules-grid">
        <div class="rule-card">
          <div class="rule-icon">
            <q-icon name="lock" size="2rem" color="red-4" />
          </div>
          <div class="rule-title">1. Cierre Obligatorio</div>
          <div class="rule-examples">
            <div class="rule-wrong"><code>&lt;link&gt;</code> sin cerrar ❌</div>
            <div class="rule-correct"><code>&lt;link&gt;&lt;/link&gt;</code> ✅</div>
          </div>
        </div>

        <div class="rule-card">
          <div class="rule-icon">
            <q-icon name="text_format" size="2rem" color="orange-4" />
          </div>
          <div class="rule-title">2. Case Sensitive</div>
          <div class="rule-examples">
            <div class="rule-wrong"><code>&lt;Robot&gt;</code> ≠ <code>&lt;robot&gt;</code> ❌</div>
            <div class="rule-correct">Usa siempre minúsculas ✅</div>
          </div>
        </div>

        <div class="rule-card">
          <div class="rule-icon">
            <q-icon name="account_tree" size="2rem" color="green-4" />
          </div>
          <div class="rule-title">3. Raíz Única</div>
          <div class="rule-examples">
            <div class="rule-wrong">Múltiples <code>&lt;robot&gt;</code> al mismo nivel ❌</div>
            <div class="rule-correct">Solo una etiqueta raíz ✅</div>
          </div>
        </div>

        <div class="rule-card">
          <div class="rule-icon">
            <q-icon name="format_quote" size="2rem" color="blue-4" />
          </div>
          <div class="rule-title">4. Comillas en Atributos</div>
          <div class="rule-examples">
            <div class="rule-wrong"><code>name=base</code> ❌</div>
            <div class="rule-correct"><code>name="base"</code> ✅</div>
          </div>
        </div>

        <div class="rule-card">
          <div class="rule-icon">
            <q-icon name="layers" size="2rem" color="purple-4" />
          </div>
          <div class="rule-title">5. Anidación Correcta</div>
          <div class="rule-examples">
            <div class="rule-wrong"><code>&lt;a&gt;&lt;b&gt;&lt;/a&gt;&lt;/b&gt;</code> ❌</div>
            <div class="rule-correct"><code>&lt;a&gt;&lt;b&gt;&lt;/b&gt;&lt;/a&gt;</code> ✅</div>
          </div>
        </div>
      </div>
    </div>

    <!-- JERARQUÍA -->
    <div class="section-group">
      <SectionTitle>3. Jerarquía: El Árbol Genealógico</SectionTitle>
      <TextBlock>
        XML es un <strong>árbol</strong>. Cada etiqueta puede tener hijos, pero no puede haber
        cruces. Piensa en un organigrama empresarial.
      </TextBlock>

      <div class="tree-demo q-mt-md">
        <div class="tree-node root">
          <div class="node-tag">&lt;robot name="R2D2"&gt;</div>
          <div class="tree-children">
            <div class="tree-node child">
              <div class="node-tag">&lt;link name="base"&gt;</div>
              <div class="tree-children">
                <div class="tree-node grandchild">
                  <div class="node-tag">&lt;visual&gt;</div>
                  <div class="tree-children">
                    <div class="tree-node leaf">
                      <div class="node-tag">&lt;geometry /&gt;</div>
                    </div>
                  </div>
                  <div class="node-tag">&lt;/visual&gt;</div>
                </div>
              </div>
              <div class="node-tag">&lt;/link&gt;</div>
            </div>

            <div class="tree-node child">
              <div class="node-tag">&lt;joint name="wheel" /&gt;</div>
            </div>
          </div>
          <div class="node-tag">&lt;/robot&gt;</div>
        </div>
      </div>
    </div>

    <!-- PACKAGE.XML -->
    <div class="section-group">
      <SectionTitle>4. package.xml: El Manifiesto del Paquete</SectionTitle>
      <TextBlock>
        Cada paquete ROS 2 tiene un <code>package.xml</code> que define su identidad y dependencias.
        Es como el <code>package.json</code> de Node.js o <code>requirements.txt</code> de Python.
      </TextBlock>

      <CodeBlock
        title="package.xml (Completo)"
        lang="xml"
        content='<?xml version="1.0"?>
<package format="3">
  <!-- METADATOS OBLIGATORIOS -->
  <name>mi_robot_control</name>
  <version>1.0.0</version>
  <description>Sistema de control para robot móvil</description>

  <!-- CONTACTO (Obligatorio) -->
  <maintainer email="alex@robot.com">Alexander</maintainer>
  <license>Apache-2.0</license>

  <!-- DEPENDENCIAS DE BUILD (compilación) -->
  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- DEPENDENCIAS DE EJECUCIÓN Y BUILD -->
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>

  <!-- SOLO EJECUCIÓN (no necesario para compilar) -->
  <exec_depend>ros2launch</exec_depend>

  <!-- SOLO BUILD (no necesario en runtime) -->
  <build_depend>rosidl_default_generators</build_depend>

  <!-- TESTING -->
  <test_depend>ament_lint_auto</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>'
        :copyable="true"
      />

      <div class="q-mt-md">
        <SectionTitle>Tipos de Dependencias</SectionTitle>
        <div class="dependencies-grid">
          <div class="dep-card">
            <code>&lt;depend&gt;</code>
            <span>Build + Ejecución (más común)</span>
          </div>
          <div class="dep-card">
            <code>&lt;build_depend&gt;</code>
            <span>Solo compilación</span>
          </div>
          <div class="dep-card">
            <code>&lt;exec_depend&gt;</code>
            <span>Solo ejecución</span>
          </div>
          <div class="dep-card">
            <code>&lt;buildtool_depend&gt;</code>
            <span>Herramientas de build (ament, colcon)</span>
          </div>
          <div class="dep-card">
            <code>&lt;test_depend&gt;</code>
            <span>Solo para tests</span>
          </div>
        </div>
      </div>
    </div>

    <!-- URDF -->
    <div class="section-group">
      <SectionTitle>5. URDF: Descripción Física del Robot</SectionTitle>
      <TextBlock>
        URDF (Unified Robot Description Format) usa XML para definir la estructura física de un
        robot: eslabones (links), articulaciones (joints), y geometrías.
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-md">
        <div class="col-12 col-md-6">
          <CodeBlock
            title="robot.urdf"
            lang="xml"
            content='<robot name="simple_robot">

  <!-- ESLABÓN (Link) -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.2" length="0.6"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder radius="0.2" length="0.6"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="10.0"/>
      <inertia ixx="1.0" iyy="1.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- ARTICULACIÓN (Joint) -->
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <axis xyz="0 1 0"/>
    <limit effort="100" velocity="10"/>
  </joint>

</robot>'
            :copyable="true"
          />
        </div>

        <div class="col-12 col-md-6">
          <div class="urdf-visual">
            <div class="urdf-title">Visualización Conceptual</div>
            <div class="robot-diagram">
              <div class="link-visual base">
                <div class="link-label">base_link</div>
                <div class="link-shape cylinder"></div>
                <div class="link-props">
                  <div>📏 radius: 0.2</div>
                  <div>📏 length: 0.6</div>
                  <div>🎨 color: blue</div>
                </div>
              </div>

              <div class="joint-visual">
                <div class="joint-line"></div>
                <div class="joint-label">wheel_joint (continuous)</div>
              </div>

              <div class="link-visual wheel">
                <div class="link-label">wheel_link</div>
                <div class="link-shape wheel"></div>
              </div>
            </div>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <AlertBlock type="info" title="Tipos de Geometría">
          <code>&lt;box size="x y z"/&gt;</code> - Caja
          <br />
          <code>&lt;cylinder radius="r" length="l"/&gt;</code> - Cilindro
          <br />
          <code>&lt;sphere radius="r"/&gt;</code> - Esfera
          <br />
          <code>&lt;mesh filename="model.stl"/&gt;</code> - Modelo 3D
        </AlertBlock>
      </div>
    </div>

    <!-- LAUNCH FILES -->
    <div class="section-group">
      <SectionTitle>6. Launch Files: Orquestando Nodos</SectionTitle>
      <TextBlock>
        Los launch files XML permiten iniciar múltiples nodos con configuraciones específicas. En
        ROS 2, también puedes usar Python launch (más flexible).
      </TextBlock>

      <CodeBlock
        title="robot_launch.xml"
        lang="xml"
        content='<launch>

  <!-- Argumentos configurables -->
  <arg name="robot_name" default="robot1"/>
  <arg name="use_sim" default="true"/>

  <!-- Nodo de control -->
  <node pkg="robot_control" exec="control_node" name="controller">
    <param name="robot_name" value="$(var robot_name)"/>
    <param name="max_speed" value="2.0"/>
  </node>

  <!-- Nodo de navegación -->
  <node pkg="robot_nav" exec="nav_node" name="navigator">
    <remap from="/cmd_vel" to="/$(var robot_name)/cmd_vel"/>
  </node>

  <!-- Incluir otro launch file -->
  <include file="$(find-pkg-share robot_sensors)/launch/sensors.launch.xml">
    <arg name="use_lidar" value="true"/>
  </include>

</launch>'
        :copyable="true"
      />

      <div class="q-mt-md">
        <AlertBlock type="success" title="XML vs Python Launch">
          <strong>XML Launch:</strong> Simple, declarativo, fácil de leer
          <br />
          <strong>Python Launch:</strong> Programático, condicionales, loops, más flexible
          <br /><br />
          Usa XML para casos simples, Python para lógica compleja.
        </AlertBlock>
      </div>
    </div>

    <!-- ERRORES COMUNES -->
    <div class="section-group">
      <SectionTitle>7. Errores Comunes y Soluciones</SectionTitle>

      <div class="errors-grid">
        <div class="error-card">
          <div class="error-header">
            <q-icon name="error" color="red-4" size="lg" />
            <span>Etiqueta no cerrada</span>
          </div>
          <div class="error-example">
            <div class="error-code wrong">
              <code>&lt;link name="base"&gt;</code>
              <br />
              <code>&nbsp;&nbsp;&lt;visual /&gt;</code>
              <br />
              <span class="error-mark">❌ Falta &lt;/link&gt;</span>
            </div>
            <div class="error-arrow">→</div>
            <div class="error-code correct">
              <code>&lt;link name="base"&gt;</code>
              <br />
              <code>&nbsp;&nbsp;&lt;visual /&gt;</code>
              <br />
              <code>&lt;/link&gt;</code> ✅
            </div>
          </div>
        </div>

        <div class="error-card">
          <div class="error-header">
            <q-icon name="error" color="orange-4" size="lg" />
            <span>Atributos sin comillas</span>
          </div>
          <div class="error-example">
            <div class="error-code wrong">
              <code>&lt;link name=base&gt;</code>
              <br />
              <span class="error-mark">❌ Falta comillas</span>
            </div>
            <div class="error-arrow">→</div>
            <div class="error-code correct"><code>&lt;link name="base"&gt;</code> ✅</div>
          </div>
        </div>

        <div class="error-card">
          <div class="error-header">
            <q-icon name="error" color="yellow-6" size="lg" />
            <span>Caracteres especiales</span>
          </div>
          <div class="error-example">
            <div class="error-code wrong">
              <code>&lt;desc&gt;5 &lt; 10&lt;/desc&gt;</code>
              <br />
              <span class="error-mark">❌ &lt; rompe el parser</span>
            </div>
            <div class="error-arrow">→</div>
            <div class="error-code correct">
              <code>&lt;desc&gt;5 &amp;lt; 10&lt;/desc&gt;</code> ✅
              <br />
              <span class="hint">Usa &amp;lt; para &lt;</span>
            </div>
          </div>
        </div>
      </div>

      <div class="q-mt-md">
        <AlertBlock type="warning" title="Entidades XML">
          <div class="entities-grid">
            <div><code>&amp;lt;</code> → <code>&lt;</code></div>
            <div><code>&amp;gt;</code> → <code>&gt;</code></div>
            <div><code>&amp;amp;</code> → <code>&amp;</code></div>
            <div><code>&amp;quot;</code> → <code>"</code></div>
            <div><code>&amp;apos;</code> → <code>'</code></div>
          </div>
        </AlertBlock>
      </div>
    </div>

    <!-- VALIDACIÓN -->
    <div class="section-group">
      <SectionTitle>8. Herramientas de Validación</SectionTitle>

      <div class="row q-col-gutter-md">
        <div class="col-12 col-md-6">
          <div class="tool-validation-card">
            <div class="tool-title">
              <q-icon name="check_circle" color="green-4" />
              <span>Validar XML Genérico</span>
            </div>
            <CodeBlock
              lang="bash"
              content="# Instalar xmllint
sudo apt install libxml2-utils

# Validar sintaxis
xmllint --noout package.xml

# Si no hay output, está correcto ✅"
              :copyable="true"
            />
          </div>
        </div>

        <div class="col-12 col-md-6">
          <div class="tool-validation-card">
            <div class="tool-title">
              <q-icon name="precision_manufacturing" color="blue-4" />
              <span>Validar URDF</span>
            </div>
            <CodeBlock
              lang="bash"
              content="# Instalar herramientas URDF
sudo apt install liburdfdom-tools

# Validar URDF
check_urdf robot.urdf

# Ver árbol de links
urdf_to_graphiz robot.urdf"
              :copyable="true"
            />
          </div>
        </div>
      </div>
    </div>

    <!-- VIDEO -->
    <div class="section-group">
      <SectionTitle>📹 Video Complementario</SectionTitle>
      <div class="video-container">
        <div class="video-wrapper">
          <iframe
            src="https://www.youtube.com/embed/Romc22GgusU"
            title="XML en ROS 2"
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
      <SectionTitle>📝 Resumen de Conceptos Clave</SectionTitle>
      <div class="summary-grid">
        <div class="summary-item">
          <code>&lt;tag&gt;</code>
          <span>Etiqueta (elemento)</span>
        </div>
        <div class="summary-item">
          <code>attr="value"</code>
          <span>Atributo con valor</span>
        </div>
        <div class="summary-item">
          <code>package.xml</code>
          <span>Manifiesto del paquete</span>
        </div>
        <div class="summary-item">
          <code>URDF</code>
          <span>Descripción física del robot</span>
        </div>
        <div class="summary-item">
          <code>launch.xml</code>
          <span>Orquestación de nodos</span>
        </div>
        <div class="summary-item">
          <code>xmllint</code>
          <span>Validador de sintaxis</span>
        </div>
      </div>

      <AlertBlock type="success" title="Checklist de XML Válido" class="q-mt-lg">
        ✅ Todas las etiquetas están cerradas
        <br />
        ✅ Atributos entre comillas dobles
        <br />
        ✅ Solo una etiqueta raíz
        <br />
        ✅ Anidación correcta (sin cruces)
        <br />
        ✅ Caracteres especiales escapados (&amp;lt;, &amp;gt;)
        <br />
        ✅ Comentarios con <code>&lt;!-- --&gt;</code>
        <br />
        ✅ Validado con <code>xmllint</code>
      </AlertBlock>
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

/* ANATOMY DEMO */
.anatomy-demo {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 3rem 2rem;
  margin-top: 1.5rem;
}

.anatomy-visual {
  display: flex;
  flex-direction: column;
  gap: 3rem;
}

.tag-breakdown {
  display: flex;
  justify-content: center;
  align-items: center;
  flex-wrap: wrap;
  gap: 0.5rem;
  font-family: 'Fira Code', monospace;
  font-size: 1.5rem;
}

.bracket {
  color: #64748b;
}

.element {
  color: #ef4444;
  font-weight: 700;
}

.attribute {
  color: #f97316;
  font-weight: 700;
}

.equals {
  color: #64748b;
}

.value {
  color: #22c55e;
}

.content {
  color: #f1f5f9;
}

.annotations {
  display: grid;
  grid-template-columns: repeat(4, 1fr);
  gap: 1rem;
}

.annotation {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.5rem;
  font-size: 0.9rem;
  font-weight: 700;
}

.element-ann {
  color: #ef4444;
}

.attribute-ann {
  color: #f97316;
}

.value-ann {
  color: #22c55e;
}

.content-ann {
  color: #f1f5f9;
}

/* COMPARISON CARDS */
.comparison-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
  height: 100%;
}

.comparison-card.attributes {
  border-top: 4px solid #f97316;
}

.comparison-card.content {
  border-top: 4px solid #3b82f6;
}

.comparison-header {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 1rem 1.5rem;
  background: rgba(0, 0, 0, 0.3);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  font-weight: 700;
  color: #f1f5f9;
}

.comparison-content {
  padding: 1.5rem;
}

.comparison-note {
  margin-top: 1rem;
  padding: 1rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 8px;
  font-size: 0.9rem;
  color: #cbd5e1;
}

.comparison-note ul {
  margin: 0.5rem 0 0 1.5rem;
  padding: 0;
}

/* RULES GRID */
.rules-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 1.5rem;
  margin-top: 1.5rem;
}

.rule-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  text-align: center;
}

.rule-icon {
  margin-bottom: 1rem;
}

.rule-title {
  font-size: 1.05rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 1rem;
}

.rule-examples {
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
  font-family: 'Fira Code', monospace;
  font-size: 0.85rem;
}

.rule-wrong {
  color: #fca5a5;
}

.rule-correct {
  color: #86efac;
}

/* TREE DEMO */
.tree-demo {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
  font-family: 'Fira Code', monospace;
}

.tree-node {
  margin-left: 2rem;
  padding: 0.5rem;
  border-left: 2px solid rgba(148, 163, 184, 0.3);
}

.tree-node.root {
  margin-left: 0;
  border-left: none;
}

.node-tag {
  color: #f1f5f9;
  padding: 0.25rem 0;
}

.tree-children {
  margin-left: 1rem;
}

.tree-node.child .node-tag {
  color: #f97316;
}

.tree-node.grandchild .node-tag {
  color: #3b82f6;
}

.tree-node.leaf .node-tag {
  color: #22c55e;
}

/* DEPENDENCIES GRID */
.dependencies-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 1rem;
  margin-top: 1rem;
}

.dep-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 8px;
  padding: 1rem;
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.dep-card code {
  font-family: 'Fira Code', monospace;
  color: #fbbf24;
  font-size: 0.9rem;
}

.dep-card span {
  color: #cbd5e1;
  font-size: 0.85rem;
}

/* URDF VISUAL */
.urdf-visual {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 2rem;
  height: 100%;
}

.urdf-title {
  font-size: 1.1rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 1.5rem;
  text-align: center;
}

.robot-diagram {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 1rem;
}

.link-visual {
  background: rgba(0, 0, 0, 0.3);
  border: 2px solid #3b82f6;
  border-radius: 12px;
  padding: 1.5rem;
  text-align: center;
  min-width: 200px;
}

.link-label {
  font-family: 'Fira Code', monospace;
  color: #60a5fa;
  font-weight: 700;
  margin-bottom: 1rem;
}

.link-shape {
  margin: 1rem auto;
}

.link-shape.cylinder {
  width: 80px;
  height: 120px;
  background: linear-gradient(90deg, #3b82f6, #60a5fa);
  border-radius: 8px;
  margin: 1rem auto;
}

.link-shape.wheel {
  width: 60px;
  height: 60px;
  background: #64748b;
  border-radius: 50%;
  margin: 1rem auto;
}

.link-props {
  font-size: 0.85rem;
  color: #cbd5e1;
  text-align: left;
}

.joint-visual {
  display: flex;
  flex-direction: column;
  align-items: center;
}

.joint-line {
  width: 2px;
  height: 40px;
  background: #fbbf24;
}

.joint-label {
  font-family: 'Fira Code', monospace;
  color: #fbbf24;
  font-size: 0.85rem;
}

/* ERRORS GRID */
.errors-grid {
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
  margin-top: 1.5rem;
}

.error-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
}

.error-header {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 1rem 1.5rem;
  background: rgba(0, 0, 0, 0.3);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  font-weight: 700;
  color: #f1f5f9;
}

.error-example {
  display: grid;
  grid-template-columns: 1fr auto 1fr;
  gap: 1rem;
  padding: 1.5rem;
  align-items: center;
}

.error-code {
  font-family: 'Fira Code', monospace;
  font-size: 0.9rem;
  padding: 1rem;
  border-radius: 8px;
}

.error-code.wrong {
  background: rgba(239, 68, 68, 0.1);
  border: 1px solid rgba(239, 68, 68, 0.3);
}

.error-code.correct {
  background: rgba(34, 197, 94, 0.1);
  border: 1px solid rgba(34, 197, 94, 0.3);
}

.error-mark {
  color: #fca5a5;
  font-size: 0.85rem;
}

.hint {
  color: #86efac;
  font-size: 0.85rem;
}

.error-arrow {
  color: #fbbf24;
  font-size: 1.5rem;
}

.entities-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
  gap: 0.5rem;
  font-family: 'Fira Code', monospace;
  font-size: 0.9rem;
}

/* TOOL VALIDATION */
.tool-validation-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
  height: 100%;
}

.tool-title {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 1rem 1.5rem;
  background: rgba(0, 0, 0, 0.3);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  font-weight: 700;
  color: #f1f5f9;
}

/* VIDEO */
.video-container {
  background: linear-gradient(135deg, rgba(15, 23, 42, 0.9), rgba(30, 41, 59, 0.9));
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
  color: #94a3b8;
  font-size: 0.85rem;
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
  color: #22c55e;
  font-size: 1rem;
}

.summary-item span {
  color: #cbd5e1;
  font-size: 0.85rem;
}

@media (max-width: 768px) {
  .annotations {
    grid-template-columns: repeat(2, 1fr);
  }

  .error-example {
    grid-template-columns: 1fr;
  }

  .error-arrow {
    transform: rotate(90deg);
  }
}
</style>

<template>
  <LessonContainer>

    <!-- ══════════════════════════════════════════
         INTRO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <TextBlock>
        Compilar ya lo sabes. Ahora toca dominar el <strong>lenguaje</strong>.
        C++ tiene características que no existen en Python — y sin entenderlas no podrás escribir
        ni tu primer nodo ROS 2 en C++. Esta lección cubre los 5 conceptos que encontrarás
        en <em>cada</em> nodo C++ de ROS 2: scope <code>::</code>, namespaces, constructores con
        initialization lists, smart pointers y lambda functions para callbacks.
      </TextBlock>

      <div class="fact-pills q-mt-lg">
        <div class="fact-pill" v-for="f in facts" :key="f.label">
          <span class="fact-icon">{{ f.icon }}</span>
          <span class="fact-label">{{ f.label }}</span>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         01 SCOPE OPERATOR & THIS
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge green">01</span>
        El Operador <code>::</code> y el Puntero <code>this</code>
      </SectionTitle>

      <TextBlock>
        Dos confusiones clásicas al venir de Python: <strong>¿Quién es este <code>::</code>?</strong>
        y <strong>¿Qué hace <code>this</code>?</strong>.
        Ambos tienen que ver con decirle al compilador "a quién pertenece esto".
      </TextBlock>

      <!-- :: scope -->
      <div class="scope-section q-mt-lg">
        <div class="sc-subtitle">
          <q-icon name="label" size="16px" color="positive" />
          El operador <code>::</code> — "pertenece a"
        </div>

        <div class="scope-grid">
          <div v-for="use in scopeUses" :key="use.label" class="scope-card"
            :style="{ '--su-color': use.color }">
            <div class="suc-label" :style="{ color: use.color }">{{ use.label }}</div>
            <CodeBlock :hide-header="true" lang="cpp" :content="use.code" />
            <div class="suc-note">{{ use.note }}</div>
          </div>
        </div>
      </div>

      <!-- this vs self -->
      <div class="this-section q-mt-xl">
        <div class="sc-subtitle">
          <q-icon name="compare" size="16px" color="warning" />
          <code>this</code> en C++ vs <code>self</code> en Python
        </div>

        <SplitBlock>
          <template #left>
            <div class="this-panel">
              <div class="thp-header thp-python">
                <q-icon name="code" size="15px" />
                Python — <code>self</code> explícito
              </div>
              <CodeBlock :hide-header="true" lang="python" :content="selfPythonCode" />
              <div class="thp-note">
                <strong>Regla:</strong> <code>self</code> es siempre el primer parámetro.
                Si olvidas escribirlo, Python falla con un error claro.
              </div>
            </div>
          </template>
          <template #right>
            <div class="this-panel">
              <div class="thp-header thp-cpp">
                <q-icon name="memory" size="15px" />
                C++ — <code>this</code> implícito
              </div>
              <CodeBlock :hide-header="true" lang="cpp" :content="thisCppCode" />
              <div class="thp-note">
                <strong>Regla:</strong> <code>this</code> siempre existe dentro de métodos,
                pero no se escribe en los parámetros. Accedes con <code>this-&gt;member</code>
                o directamente con <code>member</code>.
              </div>
            </div>
          </template>
        </SplitBlock>

        <AlertBlock type="info" title="En ROS 2 usarás this constantemente" class="q-mt-lg">
          En cada callback de ROS 2 verás <code>this-&gt;get_logger()</code>,
          <code>this-&gt;create_publisher()</code>, etc.
          Cuando pasas una función miembro como callback, debes capturar <code>this</code>
          en la lambda: <code>[this](msg) { this-&gt;mi_funcion(msg); }</code>.
        </AlertBlock>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         02 NAMESPACES
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge amber">02</span>
        Namespaces — Organizar y Evitar Conflictos
      </SectionTitle>

      <TextBlock>
        Imagina dos librerías que definen una clase llamada <code>Motor</code>.
        Sin namespaces, el compilador no sabe cuál usar — error garantizado.
        Los namespaces son "apellidos" para tus clases, funciones y constantes.
      </TextBlock>

      <!-- Conflict demo -->
      <div class="ns-conflict q-mt-lg">
        <div class="nsc-header">
          <q-icon name="warning" size="16px" color="warning" />
          El problema sin namespaces
        </div>
        <div class="nsc-panels">
          <div class="nsc-panel nsc-bad">
            <div class="nscp-label nscp-bad">Sin namespace — conflicto</div>
            <CodeBlock :hide-header="true" lang="cpp" :content="nsConflictCode" />
          </div>
          <div class="nsc-panel nsc-good">
            <div class="nscp-label nscp-good">Con namespace — sin conflicto</div>
            <CodeBlock :hide-header="true" lang="cpp" :content="nsSolvedCode" />
          </div>
        </div>
      </div>

      <!-- Namespace patterns -->
      <div class="ns-patterns q-mt-xl">
        <div class="nsp-title">
          <q-icon name="auto_stories" size="16px" color="primary" />
          Patrones comunes en ROS 2
        </div>
        <div class="nsp-grid">
          <div v-for="pat in nsPatterns" :key="pat.name" class="nsp-card"
            :style="{ '--nsp-color': pat.color }">
            <div class="nspc-name" :style="{ color: pat.color }">{{ pat.name }}</div>
            <CodeBlock :hide-header="true" lang="cpp" :content="pat.code" />
            <div class="nspc-note">{{ pat.note }}</div>
          </div>
        </div>
      </div>

      <AlertBlock type="danger" title="Nunca uses 'using namespace std;' en headers" class="q-mt-lg">
        <code>using namespace std;</code> parece cómodo pero contamina todo el código que incluya
        ese header. En los .hpp, siempre usa el nombre completo: <code>std::vector</code>,
        <code>std::string</code>. En los .cpp es tolerable, pero sigue siendo mala práctica.
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         03 CONSTRUCTORES
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">03</span>
        Constructores e Initialization Lists
      </SectionTitle>

      <TextBlock>
        El constructor es la función que se llama cuando creas un objeto.
        En C++, los miembros de la clase se inicializan <em>antes</em> de que el cuerpo del
        constructor se ejecute — y la herramienta para controlar eso es la
        <strong>initialization list</strong> (<code>: member_(val)</code>).
        En ROS 2, la llamada a <code>rclcpp::Node(name)</code> va exactamente ahí.
      </TextBlock>

      <!-- Constructor anatomy -->
      <div class="ctor-anatomy q-mt-lg">
        <div class="ca-title">
          <q-icon name="schema" size="16px" color="primary" />
          Anatomía de un constructor de nodo ROS 2
        </div>
        <CodeBlock :hide-header="true" lang="cpp" :content="ctorAnatomyCode" />

        <div class="ca-steps q-mt-lg">
          <div v-for="(step, i) in ctorSteps" :key="i" class="cas-step"
            :style="{ '--cas-color': step.color }">
            <div class="cass-num" :style="{ background: step.color }">{{ i + 1 }}</div>
            <div class="cass-body">
              <div class="cass-title">{{ step.title }}</div>
              <div class="cass-desc">{{ step.desc }}</div>
            </div>
          </div>
        </div>
      </div>

      <!-- Naming convention -->
      <div class="naming-conv q-mt-xl">
        <div class="nc-title">
          <q-icon name="spellcheck" size="16px" color="primary" />
          Convención de nombres ROS 2 — guión bajo al final
        </div>
        <SplitBlock>
          <template #left>
            <div class="nc-panel nc-bad">
              <div class="ncp-header">
                <q-icon name="warning" size="14px" color="warning" />
                Ambiguo — confunde miembro con parámetro
              </div>
              <CodeBlock :hide-header="true" lang="cpp" :content="namingBadCode" />
            </div>
          </template>
          <template #right>
            <div class="nc-panel nc-good">
              <div class="ncp-header">
                <q-icon name="check_circle" size="14px" color="positive" />
                Claro — miembros con guión bajo al final
              </div>
              <CodeBlock :hide-header="true" lang="cpp" :content="namingGoodCode" />
            </div>
          </template>
        </SplitBlock>
        <div class="nc-rule q-mt-md">
          <q-icon name="lightbulb" size="14px" color="warning" class="q-mr-sm" />
          Convención ROS 2: <code>nombre_</code> para miembros privados, <code>nombre</code>
          para parámetros. De un vistazo sabes si es local o del objeto.
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         04 SMART POINTERS
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge purple">04</span>
        Smart Pointers — Memoria Sin Fugas
      </SectionTitle>

      <TextBlock>
        En C++ clásico, gestionar memoria con <code>new</code>/<code>delete</code> manualmente
        causaba <em>memory leaks</em> y crashes. C++11 introdujo los smart pointers: objetos
        que liberan la memoria automáticamente cuando nadie más los necesita.
        En ROS 2, los usarás <strong>constantemente</strong> — los nodos y publishers son
        siempre <code>shared_ptr</code>.
      </TextBlock>

      <AlertBlock type="danger" title="Regla absoluta: nunca uses new/delete en ROS 2" class="q-mt-md">
        Todo lo que en código antiguo haría <code>Robot* r = new Robot();</code>
        en C++ moderno es <code>auto r = std::make_shared&lt;Robot&gt;();</code>.
        Los smart pointers son más seguros, más rápidos y el estándar en todo código C++ moderno.
      </AlertBlock>

      <div class="sp-cards q-mt-lg">
        <div v-for="sp in smartPointers" :key="sp.name" class="sp-card"
          :style="{ '--sp-color': sp.color }">
          <div class="spc-header">
            <div class="spc-icon-wrap" :style="{ background: sp.color + '18' }">
              <q-icon :name="sp.icon" size="24px" :style="{ color: sp.color }" />
            </div>
            <div>
              <code class="spc-name">{{ sp.name }}</code>
              <div class="spc-tagline">{{ sp.tagline }}</div>
            </div>
          </div>
          <div class="spc-desc">{{ sp.desc }}</div>
          <CodeBlock :hide-header="true" lang="cpp" :content="sp.code" />
          <div class="spc-ros">
            <q-icon name="smart_toy" size="13px" class="q-mr-xs" />
            <strong>En ROS 2:</strong> {{ sp.ros }}
          </div>
          <div v-if="sp.warning" class="spc-warning">
            <q-icon name="warning" size="13px" class="q-mr-xs" color="warning" />
            {{ sp.warning }}
          </div>
        </div>
      </div>

      <!-- Ownership visual -->
      <div class="ownership-viz q-mt-xl">
        <div class="ov-title">
          <q-icon name="account_tree" size="16px" color="primary" />
          Modelo de propiedad — quién libera la memoria
        </div>
        <div class="ov-grid">
          <div class="ov-item" v-for="ov in ownershipItems" :key="ov.ptr"
            :style="{ '--ov-color': ov.color }">
            <div class="ovi-header" :style="{ background: ov.color + '12', borderColor: ov.color + '30' }">
              <code class="ovi-ptr" :style="{ color: ov.color }">{{ ov.ptr }}</code>
            </div>
            <div class="ovi-owners">
              <div v-for="(owner, i) in ov.owners" :key="i" class="ovi-owner"
                :class="{ 'ovi-dead': owner.dead }">
                <q-icon :name="owner.dead ? 'cancel' : 'person'" size="14px"
                  :color="owner.dead ? 'negative' : 'positive'" />
                {{ owner.name }}
              </div>
            </div>
            <div class="ovi-mem">
              <div class="ovm-obj">Objeto en heap</div>
              <div class="ovm-when">Liberado cuando: {{ ov.when }}</div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         05 LAMBDA FUNCTIONS
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge red">05</span>
        Lambda Functions — El Corazón de los Callbacks ROS 2
      </SectionTitle>

      <TextBlock>
        Cada suscripción, timer y servicio en ROS 2 necesita una función callback.
        Las lambdas son funciones anónimas definidas <em>en el lugar donde se usan</em>,
        sin necesidad de declararlas por separado. Las encontrarás en cada nodo C++ de ROS 2.
      </TextBlock>

      <!-- Lambda anatomy -->
      <div class="lambda-anatomy q-mt-lg">
        <div class="la-title">
          <q-icon name="biotech" size="16px" color="primary" />
          Anatomía de una lambda
        </div>
        <div class="la-diagram">
          <div class="lad-parts">
            <div v-for="part in lambdaParts" :key="part.name" class="lad-part"
              :style="{ '--lad-color': part.color }">
              <div class="ladp-label" :style="{ background: part.color + '15', color: part.color }">
                {{ part.name }}
              </div>
              <code class="ladp-syntax">{{ part.syntax }}</code>
              <div class="ladp-desc">{{ part.desc }}</div>
            </div>
          </div>
        </div>
      </div>

      <!-- Lambda evolution -->
      <div class="lambda-levels q-mt-xl">
        <div class="ll-title">
          <q-icon name="trending_up" size="16px" color="primary" />
          De lo simple a lo completo — lambdas en ROS 2
        </div>
        <div class="ll-list">
          <div v-for="(lv, i) in lambdaLevels" :key="i" class="ll-item"
            :style="{ '--ll-color': lv.color }">
            <div class="lli-badge" :style="{ background: lv.color + '15', color: lv.color }">
              {{ lv.label }}
            </div>
            <div class="lli-body">
              <div class="lli-title">{{ lv.title }}</div>
              <CodeBlock :hide-header="true" lang="cpp" :content="lv.code" />
              <div v-if="lv.note" class="lli-note">{{ lv.note }}</div>
            </div>
          </div>
        </div>
      </div>

      <!-- Capture types -->
      <div class="capture-types q-mt-xl">
        <div class="ct-title">
          <q-icon name="camera" size="16px" color="primary" />
          Tipos de captura — lo que va entre <code>[ ]</code>
        </div>
        <div class="ct-grid">
          <div v-for="cap in captureTypes" :key="cap.syntax" class="ct-card"
            :style="{ '--ct-color': cap.color }">
            <code class="ctc-syntax">{{ cap.syntax }}</code>
            <div class="ctc-name">{{ cap.name }}</div>
            <div class="ctc-desc">{{ cap.desc }}</div>
            <div v-if="cap.ros" class="ctc-ros">
              <q-icon name="smart_toy" size="12px" class="q-mr-xs" />
              {{ cap.ros }}
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         06 CONST CORRECTNESS
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">06</span>
        Const Correctness — Promesas que el Compilador Verifica
      </SectionTitle>

      <TextBlock>
        <code>const</code> es una promesa: "esta variable/función no va a cambiar nada".
        El compilador la verifica — si mientes, falla en compilación.
        Esto detecta bugs antes de ejecutar y permite al compilador optimizar mejor.
      </TextBlock>

      <div class="const-grid q-mt-lg">
        <div v-for="cv in constVariants" :key="cv.title" class="cv-card"
          :style="{ '--cv-color': cv.color }">
          <div class="cvc-title" :style="{ color: cv.color }">{{ cv.title }}</div>
          <CodeBlock :hide-header="true" lang="cpp" :content="cv.code" />
          <div class="cvc-rule">{{ cv.rule }}</div>
        </div>
      </div>

      <AlertBlock type="info" title="Regla práctica: const por defecto" class="q-mt-lg">
        Declara todo <code>const</code> y quita el <code>const</code> solo cuando
        necesites modificarlo. Es más fácil relajar una restricción que añadirla después
        cuando el código ya asume que puede mutar todo.
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         ERRORES COMUNES
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Diccionario de Errores de Código</SectionTitle>

      <div class="error-list q-mt-lg">
        <div v-for="(err, i) in commonErrors" :key="i" class="error-item"
          :style="{ '--err-color': err.color }">
          <div class="err-header" @click="err.open = !err.open">
            <div class="err-left">
              <div class="err-num" :style="{ background: err.color + '18', color: err.color }">
                {{ i + 1 }}
              </div>
              <div>
                <div class="err-type" :style="{ color: err.color }">{{ err.type }}</div>
                <code class="err-msg">{{ err.msg }}</code>
                <div class="err-summary">{{ err.summary }}</div>
              </div>
            </div>
            <q-icon :name="err.open ? 'expand_less' : 'expand_more'" size="20px"
              style="color:var(--text-muted)" />
          </div>
          <div v-show="err.open" class="err-body">
            <div class="err-cause">
              <q-icon name="search" size="14px" class="q-mr-xs" />
              <strong>Causa:</strong> {{ err.cause }}
            </div>
            <div class="err-fix">
              <q-icon name="build" size="14px" class="q-mr-xs" color="positive" />
              <div>
                <strong>Solución:</strong>
                <ol><li v-for="s in err.steps" :key="s">{{ s }}</li></ol>
              </div>
            </div>
            <CodeBlock v-if="err.code" :hide-header="true" lang="cpp" :content="err.code" />
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         RETO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Reto — Nodo ROS 2 con Clase Propia</SectionTitle>

      <div class="challenge-box">
        <div class="challenge-header">
          <div class="challenge-icon">
            <q-icon name="emoji_events" size="28px" color="warning" />
          </div>
          <div>
            <div class="challenge-title">Escribe un nodo ROS 2 orientado a objetos</div>
            <div class="challenge-subtitle">
              Usando namespace propio, initialization list, smart pointer y lambda callback
            </div>
          </div>
          <div class="challenge-badge">50 min</div>
        </div>

        <div class="challenge-steps q-mt-md">
          <div class="cs-title">Pasos:</div>
          <div class="cs-list">
            <div v-for="step in challengeSteps" :key="step.num" class="cs-item">
              <div class="cs-num" :style="{ background: step.color }">{{ step.num }}</div>
              <div class="cs-text">{{ step.text }}</div>
            </div>
          </div>
        </div>

        <CodeBlock title="Especificación del reto" lang="cpp"
          :content="challengeCode" :copyable="true" class="q-mt-md" />

        <q-expansion-item icon="lightbulb" label="Ver pistas"
          header-class="answer-header" class="q-mt-md">
          <div class="answer-body">
            <div v-for="h in challengeHints" :key="h" class="answer-row">
              <q-icon name="chevron_right" size="14px" style="color:#4ade80" />
              {{ h }}
            </div>
          </div>
        </q-expansion-item>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         VIDEO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Video Complementario</SectionTitle>
      <TextBlock>C++ moderno para ROS 2 — smart pointers y lambdas en la práctica:</TextBlock>
      <div class="video-card q-mt-md">
        <div class="video-wrapper">
          <iframe src="https://www.youtube.com/embed/Romc22GgusU"
            title="C++ Estructura ROS 2" frameborder="0"
            allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
            allowfullscreen></iframe>
        </div>
        <div class="video-caption">
          <q-icon name="info" size="16px" color="info" class="q-mr-sm" />
          Video en progreso — será reemplazado con contenido del curso.
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         RESUMEN
    ══════════════════════════════════════════ -->
    <div class="section-group q-mb-xl">
      <SectionTitle>Resumen — Conceptos Clave</SectionTitle>
      <div class="summary-grid q-mt-lg">
        <div v-for="s in summaryItems" :key="s.cmd" class="summary-card"
          :style="{ '--sc-color': s.color }">
          <code class="sc-cmd">{{ s.cmd }}</code>
          <div class="sc-desc">{{ s.desc }}</div>
          <div class="sc-example">
            <q-icon name="arrow_right" size="14px" class="q-mr-xs" />{{ s.example }}
          </div>
        </div>
      </div>
    </div>

  </LessonContainer>
</template>

<script setup lang="ts">
import { reactive } from 'vue';
import LessonContainer from 'components/content/LessonContainer.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SplitBlock from 'components/content/SplitBlock.vue';

// ── Fact pills ─────────────────────────────────────────────────
const facts = [
  { icon: '::', label: 'Scope operator — a quién pertenece cada función' },
  { icon: '🧠', label: 'Smart pointers — memoria automática sin fugas' },
  { icon: '𝝀', label: 'Lambdas — callbacks inline sin declarar funciones extra' },
];

// ── Scope uses ─────────────────────────────────────────────────
const scopeUses = [
  {
    label: 'Implementar método fuera de la clase',
    color: '#4ade80',
    code: '// En robot.cpp\nvoid Robot::mover(double vel) {\n    posicion_ += vel;  // this->posicion_ implícito\n}',
    note: 'Obligatorio cuando el .cpp implementa lo declarado en .hpp',
  },
  {
    label: 'Acceder a namespace',
    color: '#60a5fa',
    code: 'std::cout << "Hola";  // std es namespace\nstd::vector<int> v;\nrclcpp::Node::SharedPtr nodo;',
    note: 'La STL y ROS 2 usan namespaces para evitar conflictos de nombres',
  },
  {
    label: 'Acceder a constante/tipo estático',
    color: '#c084fc',
    code: '// Acceder sin instancia\ndouble vel = Robot::MAX_VELOCIDAD;\nauto msg = std_msgs::msg::String();\n// std_msgs::msg es un namespace anidado',
    note: 'Los miembros estáticos pertenecen a la clase, no al objeto',
  },
];

// ── Namespace patterns ─────────────────────────────────────────
const nsPatterns = [
  {
    name: 'Namespace de paquete',
    color: '#4ade80',
    code: 'namespace mi_robot {\n\nclass ControlNode : public rclcpp::Node {\n    // ...\n};\n\n}  // namespace mi_robot',
    note: 'Convención ROS 2: un namespace por paquete, con el nombre del paquete',
  },
  {
    name: 'Namespace anidado (C++17)',
    color: '#60a5fa',
    code: 'namespace mi_robot::drivers {\n\nclass LidarDriver { /* ... */ };\nclass ImuDriver  { /* ... */ };\n\n}  // namespace mi_robot::drivers',
    note: 'Para organizar subsistemas. Requiere compilación con -std=c++17 o superior',
  },
  {
    name: 'using declaration (segura)',
    color: '#fbbf24',
    code: '// En .cpp (NO en .hpp)\nusing rclcpp::Node;\nusing std::shared_ptr;\n\n// Ahora puedes escribir:\nNode::SharedPtr nodo;  // en vez de rclcpp::Node::SharedPtr',
    note: 'Solo en .cpp, solo los nombres que necesitas. Nunca "using namespace std;" en .hpp',
  },
];

// ── Constructor steps ──────────────────────────────────────────
const ctorSteps = [
  { color: '#94a3b8', title: 'Se reserva memoria', desc: 'El sistema asigna espacio para todos los miembros del objeto.' },
  { color: '#fbbf24', title: 'Initialization list', desc: 'Los miembros se inicializan en el orden en que se declararon en la clase, NO en el orden de la lista.' },
  { color: '#4ade80', title: 'Clase base (Node)', desc: 'rclcpp::Node("nombre") registra el nodo con el middleware de ROS 2.' },
  { color: '#60a5fa', title: 'Cuerpo del constructor', desc: 'Aquí se crean publishers, subscriptions, timers. El nodo ya está registrado.' },
];

// ── Smart pointers ─────────────────────────────────────────────
const smartPointers = [
  {
    name: 'std::shared_ptr<T>',
    tagline: 'Propiedad compartida — múltiples dueños',
    icon: 'share',
    color: '#60a5fa',
    desc: 'Varios objetos pueden "sostener" el mismo recurso. Lleva un contador de referencias: se destruye cuando el contador llega a 0 (cuando el último dueño desaparece).',
    code: [
      '// Crear — SIEMPRE con make_shared, nunca new',
      'auto nodo = std::make_shared<MiNodo>();',
      '// Contar: 1',
      '',
      'auto copia = nodo;   // Contar: 2',
      '{',
      '    auto otra = nodo;  // Contar: 3',
      '}  // otra sale de scope → Contar: 2',
      '',
      '// copia y nodo salen de scope → Contar: 0',
      '// MiNodo se destruye automáticamente',
    ].join('\n'),
    ros: 'Los nodos ROS 2 SIEMPRE son shared_ptr. rclcpp::Node hereda de enable_shared_from_this.',
    warning: undefined,
  },
  {
    name: 'std::unique_ptr<T>',
    tagline: 'Propiedad exclusiva — un solo dueño',
    icon: 'lock',
    color: '#c084fc',
    desc: 'Solo un objeto puede poseerlo. No se puede copiar — solo transferir con std::move(). Al salir de scope, el objeto se destruye automáticamente.',
    code: [
      'auto sensor = std::make_unique<LidarSensor>(port_);',
      '',
      '// NO se puede copiar:',
      '// auto copia = sensor;   // ❌ Error de compilación',
      '',
      '// SÍ se puede mover (transferir propiedad):',
      'auto otro = std::move(sensor);',
      '// sensor ahora es nullptr',
      '// otro es el único dueño',
    ].join('\n'),
    ros: 'Drivers de hardware, recursos exclusivos. Más eficiente que shared_ptr cuando no necesitas compartir.',
    warning: undefined,
  },
  {
    name: 'std::weak_ptr<T>',
    tagline: 'Observador — sin propiedad',
    icon: 'visibility',
    color: '#4ade80',
    desc: 'Apunta al mismo objeto que un shared_ptr, pero NO cuenta como dueño. Previene referencias circulares (A→B, B→A). Para usarlo, debes convertirlo a shared_ptr con lock().',
    code: [
      'auto nodo = std::make_shared<Robot>();',
      '',
      '// weak_ptr no aumenta el contador',
      'std::weak_ptr<Robot> observador = nodo;',
      '',
      '// Para usar el objeto, hacer lock():',
      'if (auto ptr = observador.lock()) {',
      '    ptr->mover(1.0);  // objeto aún existe',
      '} else {',
      '    // El objeto ya fue destruido',
      '}',
    ].join('\n'),
    ros: 'Nodos del lifecycle que guardan referencia a otros nodos. Callbacks que pueden outlive al publisher.',
    warning: 'Si solo usas shared_ptr y hay ciclos A→B→A, ninguno se destruye nunca → memory leak.',
  },
];

// ── Ownership visual ───────────────────────────────────────────
const ownershipItems = [
  {
    ptr: 'shared_ptr',
    color: '#60a5fa',
    owners: [
      { name: 'nodo (vivo)', dead: false },
      { name: 'copia (vivo)', dead: false },
      { name: 'otra (destruida)', dead: true },
    ],
    when: 'último dueño sale de scope',
  },
  {
    ptr: 'unique_ptr',
    color: '#c084fc',
    owners: [
      { name: 'sensor (único dueño)', dead: false },
    ],
    when: 'sale de scope o se hace std::move()',
  },
  {
    ptr: 'weak_ptr',
    color: '#4ade80',
    owners: [
      { name: 'observador (no cuenta)', dead: true },
    ],
    when: 'nunca — no es dueño',
  },
];

// ── Lambda parts ───────────────────────────────────────────────
const lambdaParts = [
  { name: 'Captura', syntax: '[this, &data]', color: '#fbbf24', desc: 'Variables del scope externo que la lambda puede usar. [this] da acceso a miembros del objeto.' },
  { name: 'Parámetros', syntax: '(const std_msgs::msg::String::SharedPtr msg)', color: '#60a5fa', desc: 'Los argumentos que el caller pasa. El tipo debe coincidir con lo que el sistema espera.' },
  { name: 'Tipo retorno', syntax: '-> void', color: '#c084fc', desc: 'Opcional si el compilador puede inferirlo. Los callbacks suelen ser void.' },
  { name: 'Cuerpo', syntax: '{ RCLCPP_INFO(...); }', color: '#4ade80', desc: 'La lógica del callback. Puede acceder a todo lo capturado y a los parámetros.' },
];

// ── Lambda levels ──────────────────────────────────────────────
const lambdaLevels = [
  {
    label: 'Básica', color: '#4ade80', title: 'Timer simple — sin captura, sin parámetros',
    code: [
      '// Timer que imprime cada segundo',
      'timer_ = this->create_wall_timer(',
      '    std::chrono::seconds(1),',
      '    [this]() {',
      '        RCLCPP_INFO(this->get_logger(), "Tick!");',
      '    }',
      ');',
    ].join('\n'),
    note: '[this] captura el objeto actual para poder usar get_logger(), publishers, etc.',
  },
  {
    label: 'Media', color: '#fbbf24', title: 'Subscription — recibe un mensaje',
    code: [
      '// Suscripción a /cmd_vel',
      'sub_ = this->create_subscription<geometry_msgs::msg::Twist>(',
      '    "/cmd_vel", 10,',
      '    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {',
      '        velocidad_lineal_  = msg->linear.x;',
      '        velocidad_angular_ = msg->angular.z;',
      '        RCLCPP_INFO(get_logger(), "v=%.2f w=%.2f",',
      '                    velocidad_lineal_, velocidad_angular_);',
      '    }',
      ');',
    ].join('\n'),
    note: 'msg es SharedPtr — no lo copies, úsalo directamente. El tipo del parámetro debe coincidir con el topic.',
  },
  {
    label: 'Avanzada', color: '#c084fc', title: 'Captura por referencia — con variables externas',
    code: [
      'int contador = 0;',
      'std::string prefijo = "Robot";',
      '',
      'auto cb = [this, &contador, prefijo](const Msg::SharedPtr msg) {',
      '    contador++;              // & → modifica el original',
      '    auto p = prefijo;        // copia → solo lee',
      '    RCLCPP_INFO(get_logger(), "[%s #%d] %s",',
      '                p.c_str(), contador, msg->data.c_str());',
      '};',
    ].join('\n'),
    note: '[&var] captura por referencia (modifica el original). [var] captura por copia (solo lee).',
  },
];

// ── Capture types ──────────────────────────────────────────────
const captureTypes = [
  { syntax: '[]', name: 'Sin captura', color: '#94a3b8', desc: 'La lambda no puede acceder a nada del scope externo. Solo usa sus propios parámetros.', ros: undefined },
  { syntax: '[this]', name: 'Captura this', color: '#4ade80', desc: 'Da acceso a todos los miembros del objeto (publisher_, timer_, get_logger()...).', ros: 'El 90% de los callbacks ROS 2 usan [this]' },
  { syntax: '[=]', name: 'Captura todo por copia', color: '#fbbf24', desc: 'Copia todas las variables del scope al crear la lambda. Seguro pero puede ser costoso.', ros: undefined },
  { syntax: '[&]', name: 'Captura todo por ref', color: '#f87171', desc: 'Referencia a todo lo del scope. Cuidado: si la lambda vive más que las variables, dangling reference.', ros: 'Evitar en callbacks asíncronos de larga vida' },
];

// ── Const variants ─────────────────────────────────────────────
const constVariants = [
  {
    title: 'Variable constante', color: '#60a5fa',
    code: 'const double MAX_VEL = 2.5;  // No se puede reasignar\nconst int NUM_MOTORES = 4;\n// MAX_VEL = 3.0;  // ❌ Error de compilación',
    rule: 'Usa const para valores que no cambian nunca. El compilador puede optimizarlos.',
  },
  {
    title: 'Parámetro const reference', color: '#4ade80',
    code: '// Sin const — copia costosa:\nvoid procesar(std::string datos);\n\n// Con const& — cero copia, sin modificar:\nvoid procesar(const std::string& datos);',
    rule: 'Para pasar objetos grandes (string, vector) sin copiar Y sin modificar. El patrón más común en C++ moderno.',
  },
  {
    title: 'Método const', color: '#c084fc',
    code: 'class Robot {\npublic:\n    // Promete no modificar ningún miembro:\n    double getVelocidad() const {\n        return velocidad_;  // solo lectura\n    }\n    // void setVelocidad(double v) const  // ❌\n    //     { velocidad_ = v; }  // error: modifica miembro\nprivate:\n    double velocidad_;\n};',
    rule: 'Un método const puede llamarse en objetos constantes. Si devuelve un dato y no modifica nada, debe ser const.',
  },
];

// ── Common errors ──────────────────────────────────────────────
const commonErrors = reactive([
  {
    type: 'Error de Scope',
    msg: "error: cannot call member function without object",
    summary: 'Falta el operador :: o se intenta llamar un método no-estático como estático',
    color: '#fbbf24',
    cause: 'Estás llamando un método de instancia (no estático) sin tener un objeto, o usas el nombre del método sin indicar a qué clase pertenece en el .cpp.',
    steps: [
      'En el .cpp, usa NombreClase:: antes de cada método: void Robot::mover() { }',
      'Si necesitas llamarlo sin objeto, declara el método como static en el .hpp',
    ],
    code: '// Incorrecto — en robot.cpp:\nvoid mover() { }  // ¿de quién?\n\n// Correcto:\nvoid Robot::mover() { }',
    open: false,
  },
  {
    type: 'Error de Namespace',
    msg: "error: 'MiNodo' is not a member of 'mi_robot'",
    summary: 'Clase dentro de namespace no encontrada, o namespace incorrecto',
    color: '#60a5fa',
    cause: 'La clase está declarada dentro de namespace mi_robot pero se intenta usar sin el namespace, o el nombre del namespace está escrito diferente.',
    steps: [
      'Usa el nombre completo: mi_robot::MiNodo',
      'O agrega "using namespace mi_robot;" al inicio del .cpp (no del .hpp)',
      'Verifica que el namespace está escrito igual en el .hpp y el .cpp',
    ],
    code: '// En .hpp:\nnamespace mi_robot {\nclass MiNodo { ... };\n}  // namespace mi_robot\n\n// En .cpp:\n#include "mi_pkg/mi_nodo.hpp"\nusing namespace mi_robot;  // o usa mi_robot::MiNodo',
    open: false,
  },
  {
    type: 'Error de Smart Pointer',
    msg: "error: use of deleted function 'std::unique_ptr<T>::unique_ptr(const std::unique_ptr<T>&)'",
    summary: 'Intentaste copiar un unique_ptr — no se puede copiar, solo mover',
    color: '#c084fc',
    cause: 'unique_ptr no tiene constructor de copia por diseño. Solo puede transferirse con std::move().',
    steps: [
      'Si necesitas copiar: usa shared_ptr en lugar de unique_ptr',
      'Si necesitas transferir propiedad: usa std::move(ptr)',
    ],
    code: 'auto p1 = std::make_unique<Motor>();\n\n// ❌ Copiar — error:\n// auto p2 = p1;\n\n// ✅ Mover — transferir propiedad:\nauto p2 = std::move(p1);\n// p1 ahora es nullptr',
    open: false,
  },
  {
    type: 'Error de Lambda',
    msg: "error: 'this' was not captured for this lambda function",
    summary: 'La lambda usa miembros del objeto pero no captura this',
    color: '#f87171',
    cause: 'Dentro de la lambda usas publisher_, logger, get_logger() o cualquier miembro del nodo, pero olvidaste escribir [this] en la captura.',
    steps: [
      'Agrega [this] entre los corchetes de la lambda',
      'Si además necesitas variables locales: [this, &mi_var] o [this, mi_var]',
    ],
    code: '// ❌ Falta captura:\ntimer_ = create_wall_timer(1s, []() {\n    RCLCPP_INFO(get_logger(), "Tick");\n    //         ^^^^^^^^^^^ no capturado\n});\n\n// ✅ Con [this]:\ntimer_ = create_wall_timer(1s, [this]() {\n    RCLCPP_INFO(this->get_logger(), "Tick");\n});',
    open: false,
  },
  {
    type: 'Error de Const',
    msg: "error: passing 'const Robot' as 'this' argument discards qualifiers",
    summary: 'Se llama un método no-const en un objeto const',
    color: '#4ade80',
    cause: 'Tienes un objeto const (o referencia const) y llamas un método que no está marcado como const — el compilador no puede garantizar que el método no modifique el objeto.',
    steps: [
      'Si el método NO modifica miembros: agrega const al final: double getVel() const { return vel_; }',
      'Si el método SÍ modifica miembros: no debe ser llamado en contexto const — revisa si el objeto debería ser const',
    ],
    code: 'class Robot {\npublic:\n    // Agrega const al final — promete no modificar:\n    double getVelocidad() const { return vel_; }\n    // Nota: los setters NO pueden ser const\nprivate:\n    double vel_;\n};',
    open: false,
  },
]);

// ── Challenge ──────────────────────────────────────────────────
const challengeSteps = [
  { num: 1, color: '#60a5fa', text: 'Crea el paquete: ros2 pkg create --build-type ament_cmake mi_oop_pkg' },
  { num: 2, color: '#fbbf24', text: 'Crea include/mi_oop_pkg/sensor_node.hpp con namespace mi_oop y clase SensorNode' },
  { num: 3, color: '#4ade80', text: 'Constructor con initialization list: SensorNode() : Node("sensor"), contador_(0) { }' },
  { num: 4, color: '#c084fc', text: 'Timer con lambda [this]: imprime el valor del contador y lo incrementa' },
  { num: 5, color: '#f97316', text: 'Publisher que publica std_msgs::msg::Int32 con el valor del contador' },
  { num: 6, color: '#f87171', text: 'Método getContador() const — demuestra const correctness' },
];

const challengeCode = [
  '// Objetivo: SensorNode en namespace mi_oop',
  '// include/mi_oop_pkg/sensor_node.hpp',
  '',
  '#pragma once',
  '#include <rclcpp/rclcpp.hpp>',
  '#include <std_msgs/msg/int32.hpp>',
  '',
  'namespace mi_oop {',
  '',
  'class SensorNode : public rclcpp::Node {',
  'public:',
  '    SensorNode();',
  '    int getContador() const;  // método const',
  '',
  'private:',
  '    void timerCallback();     // implementa en .cpp',
  '',
  '    int contador_;            // guión bajo = miembro privado',
  '    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;',
  '    rclcpp::TimerBase::SharedPtr timer_;',
  '};',
  '',
  '}  // namespace mi_oop',
].join('\n');

const challengeHints = [
  'En el .cpp: #include "mi_oop_pkg/sensor_node.hpp" y using namespace mi_oop;',
  'Initialization list: SensorNode::SensorNode() : Node("sensor"), contador_(0) { ... }',
  'Timer en el constructor: timer_ = create_wall_timer(1s, [this]() { timerCallback(); });',
  'En timerCallback(): auto msg = std_msgs::msg::Int32(); msg.data = contador_++; pub_->publish(msg);',
  'getContador() const simplemente retorna contador_; — el const va DESPUÉS de los paréntesis',
  'En CMakeLists.txt: add_executable(sensor_node src/sensor_node.cpp) y ament_target_dependencies(sensor_node rclcpp std_msgs)',
];

// ── Summary ────────────────────────────────────────────────────
const summaryItems = [
  { cmd: 'Clase::metodo()', desc: 'Scope operator — implementa método fuera de la clase', example: 'void Robot::mover() { }', color: '#4ade80' },
  { cmd: 'this->miembro', desc: 'Puntero implícito al objeto actual (Python: self.attr)', example: 'this->get_logger()', color: '#60a5fa' },
  { cmd: 'namespace X { }', desc: 'Agrupa tipos para evitar conflictos de nombres', example: 'namespace mi_robot { class Node; }', color: '#fbbf24' },
  { cmd: ': base_(val)', desc: 'Initialization list — inicializa antes del cuerpo', example: 'MiNodo() : Node("nombre"), x_(0) { }', color: '#c084fc' },
  { cmd: 'make_shared<T>()', desc: 'Crea objeto con propiedad compartida', example: 'auto n = make_shared<MiNodo>();', color: '#f97316' },
  { cmd: 'make_unique<T>()', desc: 'Crea objeto con propiedad exclusiva', example: 'auto s = make_unique<Sensor>();', color: '#f87171' },
  { cmd: '[this](msg){ }', desc: 'Lambda con captura — callback de ROS 2', example: 'create_wall_timer(1s, [this](){ });', color: '#22d3ee' },
  { cmd: 'const T& param', desc: 'Referencia constante — sin copia, sin modificar', example: 'void log(const std::string& msg);', color: '#94a3b8' },
  { cmd: 'método() const', desc: 'Método que no modifica el objeto', example: 'double getVel() const { return vel_; }', color: '#4ade80' },
];

// ═══════════════════════════════════════════════════════════════
// CODE CONSTANTS
// ═══════════════════════════════════════════════════════════════

const selfPythonCode = [
  'class Robot:',
  '    # self siempre es el primer parámetro',
  '    def __init__(self, id: int):',
  '        self.id = id          # self explícito',
  '        self.speed = 0.0',
  '',
  '    def set_speed(self, speed: float):',
  '        self.speed = speed    # self explícito',
  '',
  '    def get_speed(self) -> float:',
  '        return self.speed     # self explícito',
].join('\n');

const thisCppCode = [
  'class Robot {',
  'public:',
  '    Robot(int id) : id_(id), speed_(0.0) {}',
  '    // this NO aparece en parámetros',
  '',
  '    void setSpeed(double speed) {',
  '        speed_ = speed;          // this->speed_ implícito',
  '        // o explícito:',
  '        // this->speed_ = speed;',
  '    }',
  '',
  '    double getSpeed() const {',
  '        return this->speed_;     // idéntico a: return speed_;',
  '    }',
  '',
  'private:',
  '    int id_;',
  '    double speed_;',
  '};',
].join('\n');

const nsConflictCode = [
  '// libreria_a.hpp',
  'class Motor { /* versión A */ };',
  '',
  '// libreria_b.hpp',
  'class Motor { /* versión B */ };',
  '',
  '// mi_nodo.cpp',
  '#include "libreria_a.hpp"',
  '#include "libreria_b.hpp"',
  '',
  'Motor m;  // ❌ error: ambiguous — ¿cuál Motor?',
].join('\n');

const nsSolvedCode = [
  '// libreria_a.hpp',
  'namespace lib_a {',
  '    class Motor { /* versión A */ };',
  '}',
  '',
  '// libreria_b.hpp',
  'namespace lib_b {',
  '    class Motor { /* versión B */ };',
  '}',
  '',
  '// mi_nodo.cpp',
  'lib_a::Motor motor_a;  // ✅ sin ambigüedad',
  'lib_b::Motor motor_b;  // ✅ sin ambigüedad',
].join('\n');

const ctorAnatomyCode = [
  '#include "mi_pkg/control_node.hpp"',
  '',
  'namespace mi_pkg {',
  '',
  '//           ① Nombre del constructor',
  '//           ↓',
  'ControlNode::ControlNode()',
  '    // ② Initialization list — se ejecuta ANTES del cuerpo',
  '    : rclcpp::Node("control_node"),  // ← llama al constructor base',
  '      velocidad_(0.0),               // ← inicializa miembro',
  '      max_vel_(2.5),                 // ← inicializa constante',
  '      activo_(false)                 // ← inicializa flag',
  '{',
  '    // ③ Cuerpo — el nodo ya está registrado en ROS 2',
  '    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(',
  '        "/cmd_vel", 10',
  '    );',
  '',
  '    timer_ = this->create_wall_timer(',
  '        std::chrono::milliseconds(100),',
  '        [this]() { timerCallback(); }',
  '    );',
  '',
  '    RCLCPP_INFO(this->get_logger(), "ControlNode iniciado");',
  '}',
  '',
  '}  // namespace mi_pkg',
].join('\n');

const namingBadCode = [
  'class Robot {',
  'public:',
  '    Robot(int id) {',
  '        // ¿id es el parámetro o el miembro?',
  '        id = id;   // ❌ se asigna a sí mismo',
  '    }',
  'private:',
  '    int id;  // miembro con mismo nombre que parámetro',
  '};',
].join('\n');

const namingGoodCode = [
  'class Robot {',
  'public:',
  '    Robot(int id)         // parámetro: sin guión',
  '        : id_(id) {}     // miembro: con guión al final',
  '',
  '    // Nunca hay ambigüedad:',
  '    void setId(int id) {',
  '        id_ = id;  // id_ es miembro, id es parámetro',
  '    }',
  'private:',
  '    int id_;     // ← convención ROS 2: nombre_',
  '    double vel_; // ← siempre guión bajo al final',
  '};',
].join('\n');
</script>

<style scoped>
/* ══════════════════════════════════════════
   BASE
══════════════════════════════════════════ */
.section-group { margin-bottom: 3.5rem; }

code {
  background: var(--bg-code); color: var(--text-code);
  padding: 2px 7px; border-radius: 5px;
  font-family: 'Fira Code', monospace; font-size: .9em;
}
.cmd-badge {
  display: inline-flex; align-items: center; justify-content: center;
  width: 28px; height: 28px; border-radius: 8px;
  font-size: .75rem; font-weight: 800; margin-right: 8px; vertical-align: middle;
}
.cmd-badge.green  { background: rgba( 74,222,128,.15); color: #4ade80; }
.cmd-badge.amber  { background: rgba(251,191, 36,.15); color: #fbbf24; }
.cmd-badge.cyan   { background: rgba( 34,211,238,.15); color: #22d3ee; }
.cmd-badge.purple { background: rgba(192,132,252,.15); color: #c084fc; }
.cmd-badge.red    { background: rgba(248,113,113,.15); color: #f87171; }

.fact-pills { display: flex; gap: 10px; flex-wrap: wrap; }
.fact-pill {
  display: flex; align-items: center; gap: 8px;
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 999px; padding: 7px 16px; font-size: .84rem; color: var(--text-secondary);
  transition: transform .2s;
}
.fact-pill:hover { transform: translateY(-2px); }
.fact-icon { font-size: 1rem; font-family: monospace; }

.sc-subtitle {
  display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 700;
  color: var(--text-secondary); margin-bottom: 12px;
}

/* ══════════════════════════════════════════
   SCOPE
══════════════════════════════════════════ */
.scope-grid { display: grid; grid-template-columns: repeat(3,1fr); gap: 12px; }
.scope-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--su-color, var(--border-medium));
  border-radius: 12px; padding: 1rem; display: flex; flex-direction: column; gap: 8px;
  min-width: 0;
}
.suc-label { font-size: .82rem; font-weight: 700; }
.suc-note  { font-size: .77rem; color: var(--text-muted); line-height: 1.4; background: var(--bg-surface-hover); border-radius: 6px; padding: 6px 9px; }

/* this vs self */
.this-section { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 14px; padding: 1.5rem; }
.this-panel { display: flex; flex-direction: column; gap: 8px; }
.thp-header {
  display: flex; align-items: center; gap: 7px; font-size: .84rem; font-weight: 700;
  padding: 8px 12px; border-radius: 8px;
}
.thp-python { background: rgba(251,191,36,.1); color: #fbbf24; }
.thp-cpp    { background: rgba( 96,165,250,.1); color: #60a5fa; }
.thp-note   { font-size: .81rem; color: var(--text-secondary); background: var(--bg-surface-hover); border-radius: 7px; padding: 8px 12px; line-height: 1.4; }

/* ══════════════════════════════════════════
   NAMESPACES
══════════════════════════════════════════ */
.ns-conflict { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 14px; overflow: hidden; }
.nsc-header  { display: flex; align-items: center; gap: 8px; padding: 12px 16px; border-bottom: 1px solid var(--border-subtle); font-size: .88rem; font-weight: 600; color: var(--text-secondary); background: var(--bg-surface-solid); }
.nsc-panels  { display: grid; grid-template-columns: repeat(2,1fr); }
.nsc-panel   { padding: 12px; min-width: 0; }
.nsc-bad     { border-right: 1px solid var(--border-subtle); }
.nscp-label  { font-size: .82rem; font-weight: 700; margin-bottom: 8px; padding: 5px 10px; border-radius: 7px; }
.nscp-bad    { background: rgba(248,113,113,.1); color: #f87171; }
.nscp-good   { background: rgba( 74,222,128,.1); color: #4ade80; }

.ns-patterns { }
.nsp-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }
.nsp-grid  { display: grid; grid-template-columns: repeat(3,1fr); gap: 12px; }
.nsp-card  {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 3px solid var(--nsp-color, var(--border-medium));
  border-radius: 12px; padding: 1rem; display: flex; flex-direction: column; gap: 8px; min-width: 0;
}
.nspc-name { font-size: .85rem; font-weight: 700; }
.nspc-note { font-size: .78rem; color: var(--text-muted); background: var(--bg-surface-hover); border-radius: 6px; padding: 5px 9px; line-height: 1.4; }

/* ══════════════════════════════════════════
   CONSTRUCTORES
══════════════════════════════════════════ */
.ctor-anatomy { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 14px; padding: 1.5rem; }
.ca-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }
.ca-steps { display: flex; flex-direction: column; gap: 8px; }
.cas-step {
  display: flex; align-items: flex-start; gap: 12px;
  background: var(--bg-surface-hover); border-left: 3px solid var(--cas-color);
  border-radius: 8px; padding: 10px 12px; min-width: 0;
}
.cass-num {
  min-width: 30px; width: 30px; height: 30px; border-radius: 50%; flex-shrink: 0;
  font-size: .85rem; font-weight: 800; color: #1e1e1e;
  display: flex; align-items: center; justify-content: center;
}
.cass-title { font-size: .87rem; font-weight: 700; color: var(--text-primary); margin-bottom: 3px; }
.cass-desc  { font-size: .8rem; color: var(--text-secondary); line-height: 1.4; }

.naming-conv { }
.nc-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }
.nc-panel { display: flex; flex-direction: column; gap: 8px; }
.ncp-header {
  display: flex; align-items: center; gap: 7px; font-size: .84rem; font-weight: 700;
  padding: 8px 12px; border-radius: 8px;
}
.nc-bad  .ncp-header { background: rgba(248,113,113,.1); color: #f87171; }
.nc-good .ncp-header { background: rgba( 74,222,128,.1); color: #4ade80; }
.nc-rule {
  display: flex; align-items: flex-start; gap: 6px; font-size: .82rem; color: var(--text-secondary);
  background: rgba(251,191,36,.08); border: 1px solid rgba(251,191,36,.25);
  border-radius: 8px; padding: 8px 12px;
}

/* ══════════════════════════════════════════
   SMART POINTERS
══════════════════════════════════════════ */
.sp-cards { display: flex; flex-direction: column; gap: 16px; }
.sp-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 4px solid var(--sp-color, var(--border-medium));
  border-radius: 14px; padding: 1.25rem; display: flex; flex-direction: column; gap: 10px;
}
.spc-header { display: flex; align-items: center; gap: 12px; }
.spc-icon-wrap { width: 44px; height: 44px; border-radius: 10px; display: flex; align-items: center; justify-content: center; flex-shrink: 0; }
.spc-name    { font-family: 'Fira Code', monospace; font-size: 1rem; font-weight: 700; color: var(--sp-color); background: none; padding: 0; display: block; }
.spc-tagline { font-size: .82rem; color: var(--text-muted); margin-top: 2px; }
.spc-desc    { font-size: .84rem; color: var(--text-secondary); line-height: 1.5; }
.spc-ros {
  display: flex; align-items: flex-start; font-size: .8rem; color: var(--text-secondary);
  background: var(--bg-surface-hover); border-radius: 7px; padding: 7px 10px;
}
.spc-warning {
  display: flex; align-items: flex-start; font-size: .8rem; color: var(--text-secondary);
  background: rgba(251,191,36,.07); border: 1px solid rgba(251,191,36,.2);
  border-radius: 7px; padding: 7px 10px;
}

.ownership-viz { }
.ov-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }
.ov-grid  { display: grid; grid-template-columns: repeat(3,1fr); gap: 14px; }
.ov-item  {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 12px; overflow: hidden; min-width: 0;
}
.ovi-header  { padding: 10px 12px; border-bottom: 1px solid; }
.ovi-ptr     { font-family: 'Fira Code', monospace; font-size: .88rem; font-weight: 700; background: none; padding: 0; }
.ovi-owners  { padding: 10px 12px; display: flex; flex-direction: column; gap: 6px; border-bottom: 1px solid var(--border-subtle); }
.ovi-owner   { display: flex; align-items: center; gap: 6px; font-size: .81rem; color: var(--text-secondary); }
.ovi-dead    { opacity: .5; text-decoration: line-through; }
.ovi-mem     { padding: 8px 12px; }
.ovm-obj     { font-size: .78rem; font-weight: 700; color: var(--text-secondary); margin-bottom: 2px; }
.ovm-when    { font-size: .74rem; color: var(--text-muted); }

/* ══════════════════════════════════════════
   LAMBDAS
══════════════════════════════════════════ */
.lambda-anatomy { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 14px; padding: 1.5rem; }
.la-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }
.la-diagram { }
.lad-parts { display: grid; grid-template-columns: repeat(4,1fr); gap: 10px; }
.lad-part  { background: var(--bg-surface-hover); border-radius: 10px; padding: 10px; min-width: 0; display: flex; flex-direction: column; gap: 6px; }
.ladp-label  { font-size: .72rem; font-weight: 800; letter-spacing: .06em; padding: 3px 8px; border-radius: 999px; display: inline-block; align-self: flex-start; }
.ladp-syntax { font-family: 'Fira Code', monospace; font-size: .78rem; font-weight: 700; background: none; padding: 0; color: var(--text-primary); word-break: break-word; }
.ladp-desc   { font-size: .76rem; color: var(--text-muted); line-height: 1.4; }

.lambda-levels { }
.ll-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }
.ll-list  { display: flex; flex-direction: column; gap: 12px; }
.ll-item  {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 3px solid var(--ll-color, var(--border-medium));
  border-radius: 12px; padding: 1rem 1.25rem; display: flex; gap: 14px; min-width: 0;
}
.lli-badge {
  flex-shrink: 0; align-self: flex-start; font-size: .72rem; font-weight: 800;
  letter-spacing: .06em; padding: 3px 10px; border-radius: 999px; white-space: nowrap;
}
.lli-body  { flex: 1; min-width: 0; display: flex; flex-direction: column; gap: 7px; }
.lli-title { font-size: .87rem; font-weight: 700; color: var(--text-primary); }
.lli-note  { font-size: .79rem; color: var(--text-muted); background: var(--bg-surface-hover); border-radius: 6px; padding: 6px 10px; }

.capture-types { }
.ct-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }
.ct-grid  { display: grid; grid-template-columns: repeat(4,1fr); gap: 10px; }
.ct-card  {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--ct-color, var(--border-medium));
  border-radius: 10px; padding: 10px 12px; display: flex; flex-direction: column; gap: 6px; min-width: 0;
}
.ctc-syntax { font-family: 'Fira Code', monospace; font-size: .92rem; font-weight: 700; color: var(--ct-color); background: none; padding: 0; }
.ctc-name   { font-size: .8rem; font-weight: 600; color: var(--text-primary); }
.ctc-desc   { font-size: .77rem; color: var(--text-secondary); line-height: 1.4; }
.ctc-ros    { font-size: .73rem; color: var(--text-muted); background: var(--bg-surface-hover); border-radius: 5px; padding: 4px 7px; display: flex; align-items: center; }

/* ══════════════════════════════════════════
   CONST VARIANTS
══════════════════════════════════════════ */
.const-grid { display: grid; grid-template-columns: repeat(3,1fr); gap: 14px; }
.cv-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--cv-color, var(--border-medium));
  border-radius: 12px; padding: 1rem; display: flex; flex-direction: column; gap: 8px; min-width: 0;
}
.cvc-title { font-size: .87rem; font-weight: 700; }
.cvc-rule  { font-size: .78rem; color: var(--text-muted); background: var(--bg-surface-hover); border-radius: 6px; padding: 6px 9px; line-height: 1.4; }

/* ══════════════════════════════════════════
   ERROR LIST
══════════════════════════════════════════ */
.error-list { display: flex; flex-direction: column; gap: 10px; }
.error-item {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 3px solid var(--err-color, #ef4444); border-radius: 12px; overflow: hidden;
}
.err-header {
  display: flex; align-items: center; justify-content: space-between;
  padding: .9rem 1.25rem; cursor: pointer; gap: 12px; transition: background .2s;
}
.err-header:hover { background: var(--bg-surface-hover); }
.err-left    { display: flex; align-items: flex-start; gap: 10px; min-width: 0; }
.err-num     { min-width: 26px; width: 26px; height: 26px; border-radius: 50%; flex-shrink: 0; font-size: .8rem; font-weight: 800; display: flex; align-items: center; justify-content: center; }
.err-type    { font-size: .7rem; font-weight: 700; letter-spacing: .05em; text-transform: uppercase; margin-bottom: 1px; }
.err-msg     { font-size: .82rem; display: block; margin-bottom: 2px; color: #f87171; background: none; padding: 0; word-break: break-all; }
.err-summary { font-size: .78rem; color: var(--text-muted); }
.err-body    { padding: .9rem 1.4rem 1.1rem; border-top: 1px solid var(--border-subtle); display: flex; flex-direction: column; gap: 10px; }
.err-cause, .err-fix { font-size: .86rem; color: var(--text-secondary); display: flex; align-items: flex-start; gap: 6px; }
.err-fix ol  { margin: 4px 0 0 14px; }
.err-fix li  { margin-bottom: 4px; }

/* ══════════════════════════════════════════
   CHALLENGE BOX
══════════════════════════════════════════ */
.challenge-box {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 20px; padding: 1.75rem; border-top: 3px solid #f59e0b;
}
.challenge-header  { display: flex; align-items: flex-start; gap: 1rem; flex-wrap: wrap; }
.challenge-icon    { width: 52px; height: 52px; background: rgba(245,158,11,.15); border-radius: 14px; display: flex; align-items: center; justify-content: center; flex-shrink: 0; }
.challenge-title   { font-size: 1.05rem; font-weight: 700; color: var(--text-primary); margin-bottom: 4px; }
.challenge-subtitle { font-size: .9rem; color: var(--text-secondary); }
.challenge-badge   { margin-left: auto; font-size: .72rem; font-weight: 800; letter-spacing: .07em; padding: 4px 12px; border-radius: 999px; white-space: nowrap; background: rgba(96,165,250,.12); color: #60a5fa; border: 1px solid rgba(96,165,250,.3); }
.challenge-steps   { background: var(--bg-surface-hover); border-radius: 12px; padding: 1rem 1.25rem; }
.cs-title  { font-size: .88rem; font-weight: 700; color: var(--text-secondary); margin-bottom: 10px; }
.cs-list   { display: flex; flex-direction: column; gap: 8px; }
.cs-item   { display: flex; align-items: flex-start; gap: 10px; }
.cs-num    { min-width: 26px; width: 26px; height: 26px; border-radius: 50%; flex-shrink: 0; font-size: .82rem; font-weight: 800; color: #1e1e1e; display: flex; align-items: center; justify-content: center; }
.cs-text   { font-size: .87rem; color: var(--text-secondary); padding-top: 3px; }
:deep(.answer-header) { background: rgba(34,197,94,.08); border: 1px solid rgba(34,197,94,.25); border-radius: 10px; color: #22c55e; }
.answer-body { background: var(--bg-surface-hover); padding: 1.1rem 1.25rem; border-radius: 0 0 10px 10px; display: flex; flex-direction: column; gap: 8px; }
.answer-row  { display: flex; align-items: baseline; gap: 8px; font-size: .88rem; color: var(--text-secondary); }

/* ══════════════════════════════════════════
   VIDEO
══════════════════════════════════════════ */
.video-card    { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 16px; padding: 1.25rem; overflow: hidden; }
.video-wrapper { position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; border-radius: 10px; background: #000; }
.video-wrapper iframe { position: absolute; top: 0; left: 0; width: 100%; height: 100%; }
.video-caption { display: flex; align-items: center; margin-top: 12px; font-size: .82rem; color: var(--text-muted); padding: 8px 12px; background: var(--bg-surface-hover); border-radius: 8px; }

/* ══════════════════════════════════════════
   SUMMARY
══════════════════════════════════════════ */
.summary-grid { display: grid; grid-template-columns: repeat(3,1fr); gap: 14px; }
.summary-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 4px solid var(--sc-color, var(--border-medium));
  border-radius: 12px; padding: 1rem 1.25rem; transition: all .25s;
}
.summary-card:hover { transform: translateY(-3px); box-shadow: var(--shadow-sm); }
.sc-cmd     { display: block; font-family: 'Fira Code', monospace; font-size: .88rem; font-weight: 700; color: var(--sc-color); background: none; padding: 0; margin-bottom: 5px; }
.sc-desc    { font-size: .81rem; color: var(--text-secondary); margin-bottom: 6px; line-height: 1.4; }
.sc-example { display: flex; align-items: center; font-family: 'Fira Code', monospace; font-size: .72rem; color: var(--text-muted); }

/* ══════════════════════════════════════════
   RESPONSIVE
══════════════════════════════════════════ */
@media (max-width: 1024px) {
  .lad-parts { grid-template-columns: repeat(2,1fr); }
  .ct-grid   { grid-template-columns: repeat(2,1fr); }
}
@media (max-width: 900px) {
  .scope-grid  { grid-template-columns: 1fr; }
  .nsp-grid    { grid-template-columns: 1fr; }
  .nsc-panels  { grid-template-columns: 1fr; }
  .nsc-bad     { border-right: none; border-bottom: 1px solid var(--border-subtle); }
  .const-grid  { grid-template-columns: 1fr; }
  .ov-grid     { grid-template-columns: 1fr; }
  .summary-grid { grid-template-columns: repeat(2,1fr); }
}
@media (max-width: 768px) {
  .lad-parts       { grid-template-columns: 1fr; }
  .ct-grid         { grid-template-columns: 1fr; }
  .challenge-header { flex-direction: column; }
  .challenge-badge { margin-left: 0; }
  .ll-item         { flex-direction: column; }
}
@media (max-width: 480px) {
  .summary-grid { grid-template-columns: 1fr; }
}
</style>

<template>
  <div class="dependency-solver">
    <div class="solver-header">
      <div class="header-title">
        <q-icon name="bug_report" color="red-4" />
        Dependency Hell Solver
      </div>
      <div class="header-score">
        Score: <span class="text-green-4">{{ score }}/3</span>
      </div>
    </div>

    <div class="solver-content" v-if="currentLevel <= totalLevels">
      <div class="scenario-box">
        <div class="scenario-text">
          <span class="text-grey-5">Scenario {{ currentLevel }}:</span>
          <br />
          {{ currentScenario.description }}
        </div>
        <div class="code-snippet">
          <pre><code>{{ currentScenario.codeSnippet }}</code></pre>
        </div>
      </div>

      <div class="problem-statement">
        El paquete falla al compilar/ejecutar. Falta la dependencia
        <code class="text-secondary">{{ currentScenario.missingPkg }}</code
        >.
        <br />
        ¿Qué tag debes añadir al <code>package.xml</code>?
      </div>

      <div class="options-grid">
        <button
          v-for="opt in options"
          :key="opt"
          class="option-btn"
          @click="checkAnswer(opt)"
          :disabled="showResult"
        >
          &lt;{{ opt }}&gt;
        </button>
      </div>

      <div v-if="showResult" class="result-box" :class="isCorrect ? 'correct' : 'incorrect'">
        <div class="result-icon">
          <q-icon :name="isCorrect ? 'check_circle' : 'cancel'" size="md" />
        </div>
        <div class="result-text">
          <div class="result-title">{{ isCorrect ? '¡Correcto!' : 'Incorrecto' }}</div>
          <div class="result-expl">{{ currentScenario.explanation }}</div>
        </div>
        <q-btn
          class="next-btn"
          :color="isCorrect ? 'green' : 'grey'"
          label="Siguiente >"
          @click="nextLevel"
          unelevated
        />
      </div>
    </div>

    <div v-else class="completion-screen">
      <q-icon name="emoji_events" size="4rem" color="yellow-4" />
      <div class="completion-title">¡Workspace Configurado!</div>
      <div class="completion-desc">Has resuelto correctamente las dependencias del sistema.</div>
      <q-btn color="primary" label="Reiniciar" @click="resetGame" outline class="q-mt-md" />
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, computed } from 'vue';

interface Scenario {
  description: string;
  codeSnippet: string;
  missingPkg: string;
  correctTag: string;
  explanation: string;
}

const score = ref(0);
const currentLevel = ref(1);
const showResult = ref(false);
const isCorrect = ref(false);
const totalLevels = 3;

const options = ['build_depend', 'exec_depend', 'test_depend', 'depend'];

const scenarios: Scenario[] = [
  {
    description: 'Estás escribiendo un nodo en C++ que usa un mensaje estándar.',
    codeSnippet:
      '#include <geometry_msgs/msg/twist.hpp>\n\nvoid move() {\n  geometry_msgs::msg::Twist msg;\n  // ...\n}',
    missingPkg: 'geometry_msgs',
    correctTag: 'depend',
    explanation:
      'Para mensajes, necesitas los headers al compilar (build) y las librerías al ejecutar (exec). <depend> cubre ambos.',
  },
  {
    description: 'Tu nodo usa una librería de Python para cálculos numéricos solo en runtime.',
    codeSnippet: 'import numpy as np\n\ndef calculate():\n    return np.random.rand(3,3)',
    missingPkg: 'python3-numpy',
    correctTag: 'exec_depend',
    explanation:
      'En Python o librerías dinámicas puras, solo necesitas la dependencia en tiempo de ejecución.',
  },
  {
    description: 'Estás usando gtest para verificar tu código en la fase de test.',
    codeSnippet: '#include <gtest/gtest.h>\n\nTEST(MyNode, SimpleTest) {\n  EXPECT_TRUE(true);\n}',
    missingPkg: 'ament_cmake_gtest',
    correctTag: 'test_depend',
    explanation:
      'Las dependencias de prueba no deben ensuciar el entorno de producción. Usa <test_depend>.',
  },
];

const currentScenario = computed<Scenario>(() => {
  // Explicit cast to assure TS that we always return a Scenario
  const scen = scenarios[currentLevel.value - 1];
  return (scen || scenarios[0]) as Scenario;
});

const checkAnswer = (tag: string) => {
  const correct = currentScenario.value.correctTag;
  isCorrect.value = tag === correct;

  if (isCorrect.value) score.value++;
  showResult.value = true;
};

const nextLevel = () => {
  currentLevel.value++;
  showResult.value = false;
  isCorrect.value = false;
};

const resetGame = () => {
  score.value = 0;
  currentLevel.value = 1;
  showResult.value = false;
};
</script>

<style scoped>
.dependency-solver {
  background: rgba(15, 23, 42, 0.9);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
  max-width: 600px;
  margin: 2rem auto;
  font-family: 'Inter', sans-serif;
}

.solver-header {
  padding: 1rem;
  background: rgba(0, 0, 0, 0.4);
  display: flex;
  justify-content: space-between;
  align-items: center;
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
}

.header-title {
  font-weight: 700;
  color: #f1f5f9;
  display: flex;
  align-items: center;
  gap: 0.5rem;
}

.solver-content,
.completion-screen {
  padding: 2rem;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.completion-screen {
  align-items: center;
  text-align: center;
}

.completion-title {
  font-size: 1.5rem;
  font-weight: 700;
  color: #f1f5f9;
}

.completion-desc {
  color: #94a3b8;
}

.scenario-box {
  background: rgba(30, 41, 59, 0.5);
  border-radius: 8px;
  padding: 1rem;
  border: 1px solid rgba(148, 163, 184, 0.1);
}

.scenario-text {
  color: #e2e8f0;
  font-size: 0.95rem;
  margin-bottom: 0.5rem;
}

.code-snippet {
  background: #0f172a;
  padding: 0.75rem;
  border-radius: 6px;
  border-left: 3px solid #3b82f6;
  overflow-x: auto;
}

.code-snippet pre {
  margin: 0;
  font-family: 'Fira Code', monospace;
  font-size: 0.85rem;
  color: #a5b4fc;
}

.problem-statement {
  font-size: 1rem;
  color: #cbd5e1;
  text-align: center;
}

.options-grid {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 1rem;
}

.option-btn {
  background: rgba(30, 41, 59, 0.8);
  border: 1px solid rgba(148, 163, 184, 0.3);
  color: #60a5fa;
  padding: 0.75rem;
  border-radius: 6px;
  cursor: pointer;
  font-family: 'Fira Code', monospace;
  font-weight: 600;
  transition: all 0.2s;
}

.option-btn:hover:not(:disabled) {
  background: rgba(59, 130, 246, 0.2);
  border-color: #3b82f6;
}

.option-btn:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}

.result-box {
  background: rgba(0, 0, 0, 0.3);
  padding: 1rem;
  border-radius: 8px;
  display: flex;
  gap: 1rem;
  align-items: center;
  animation: slide-up 0.3s ease;
}

.result-box.correct {
  border: 1px solid #22c55e;
  background: rgba(34, 197, 94, 0.1);
}

.result-box.incorrect {
  border: 1px solid #ef4444;
  background: rgba(239, 68, 68, 0.1);
}

.result-text {
  flex: 1;
}

.result-title {
  font-weight: 700;
  color: #f1f5f9;
}

.result-expl {
  font-size: 0.85rem;
  color: #cbd5e1;
  margin-top: 0.25rem;
}

@keyframes slide-up {
  from {
    transform: translateY(10px);
    opacity: 0;
  }
  to {
    transform: translateY(0);
    opacity: 1;
  }
}
</style>

import type { RouteRecordRaw } from 'vue-router';
import MainLayout from 'layouts/MainLayout.vue';
import type { CourseNode } from 'src/data/courseStructure';
import { courseStructure } from 'src/data/courseStructure';

/*
  GENERADOR DE RUTAS ADAPTADO A TU ESTRUCTURA
  -------------------------------------------
  Convención Actual:
  - URL: /modulo-0/nav-sistema
  - Archivo Físico: src/pages/course/modulo0/nav-sistema.vue
*/
const generateRoutesFromStructure = (nodes: CourseNode[]): RouteRecordRaw[] => {
  const generatedRoutes: RouteRecordRaw[] = [];

  nodes.forEach((node) => {
    // Caso 1: Módulos (ej: modulo-0)
    if (node.children && node.children.length > 0) {
      // Convertimos 'modulo-0' a 'modulo0' (sin guion)
      const folderName = node.path?.replace('-', '') || '';

      node.children.forEach((child) => {
        // IMPORTANTE: Usamos el path tal cual está en el JSON (kebab-case)
        // Ejemplo: 'nav-sistema' -> busca 'nav-sistema.vue'
        const fileName = child.path;

        if (folderName && fileName) {
          generatedRoutes.push({
            path: `${node.path}/${child.path}`,
            name: `${folderName}-${child.path}`,
            component: () => import(`pages/course/${folderName}/${fileName}.vue`),
          });
        }
      });
    }
    // Caso 2: Páginas sueltas
    else if (node.path) {
      // Convierte 'dashboard' a 'Dashboard'
      const fileName = node.path.charAt(0).toUpperCase() + node.path.slice(1);

      generatedRoutes.push({
        path: node.path,
        name: node.path,
        component: () => import(`pages/${fileName}.vue`),
      });
    }
  });

  return generatedRoutes;
};

const dynamicRoutes = generateRoutesFromStructure(courseStructure);

const routes: RouteRecordRaw[] = [
  {
    path: '/',
    component: MainLayout,
    children: [
      { path: '', redirect: '/home' },

      { path: 'home', component: () => import('pages/HomePage.vue') },
      ...dynamicRoutes,
      { path: 'glosario', component: () => import('pages/GlosarioPage.vue') },
      { path: 'creditos', component: () => import('pages/CreditosPage.vue') },
      { path: 'plantilla', component: () => import('pages/PlantillaPage.vue') },
      { path: 'introduccion', component: () => import('pages/IntroduccionPage.vue') },
      { path: 'instalacion', component: () => import('pages/InstalacionPage.vue') },
      { path: 'fundamentos', component: () => import('pages/FundamentosPage.vue') },
      { path: 'simulacion', component: () => import('pages/SimulacionPage.vue') },
    ],
  },

  {
    path: '/:catchAll(.*)*',
    component: () => import('pages/ErrorNotFound.vue'),
  },
];

export default routes;

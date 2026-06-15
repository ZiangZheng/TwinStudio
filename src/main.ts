import './style.css';
import { runApp } from './app';

const app = document.getElementById('app');
if (!app) throw new Error('Missing #app root.');

runApp(app).catch((error) => {
  console.error(error);
  app.innerHTML = `
    <main class="fatal">
      <h1>SonicTwin Studio failed to start</h1>
      <pre>${error instanceof Error ? error.message : String(error)}</pre>
    </main>
  `;
});

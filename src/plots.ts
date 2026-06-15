import {
  CategoryScale,
  Chart,
  Filler,
  Legend,
  LineController,
  LineElement,
  LinearScale,
  PointElement,
  Title,
  Tooltip,
} from 'chart.js';

Chart.register(LineController, LineElement, PointElement, LinearScale, CategoryScale, Title, Tooltip, Legend, Filler);

export class RealTimePlot {
  private readonly chart: Chart<'line'>;

  constructor(
    canvas: HTMLCanvasElement,
    title: string,
    labels: string[],
    colors: string[],
    private readonly maxPoints = 180,
  ) {
    this.chart = new Chart(canvas, {
      type: 'line',
      data: {
        labels: [],
        datasets: labels.map((label, index) => ({
          label,
          data: [],
          borderColor: colors[index % colors.length],
          backgroundColor: `${colors[index % colors.length]}24`,
          borderWidth: 1.7,
          pointRadius: 0,
          tension: 0.28,
          fill: false,
        })),
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        interaction: { mode: 'index', intersect: false },
        plugins: {
          title: {
            display: true,
            text: title,
            align: 'start',
            color: '#d8e3f1',
            font: { size: 12, weight: 600 },
          },
          legend: {
            align: 'end',
            labels: { color: '#92a2b7', boxWidth: 10, boxHeight: 10, font: { size: 10 } },
          },
          tooltip: { enabled: true },
        },
        scales: {
          x: {
            ticks: { color: '#6f8198', maxTicksLimit: 5, font: { size: 10 } },
            grid: { color: 'rgba(255,255,255,0.04)' },
          },
          y: {
            ticks: { color: '#6f8198', font: { size: 10 } },
            grid: { color: 'rgba(255,255,255,0.06)' },
          },
        },
      },
    });
  }

  push(values: number[], label: string): void {
    const labels = this.chart.data.labels as string[];
    labels.push(label);
    if (labels.length > this.maxPoints) labels.shift();

    values.forEach((value, index) => {
      const dataset = this.chart.data.datasets[index];
      if (!dataset) return;
      dataset.data.push(value);
      if (dataset.data.length > this.maxPoints) dataset.data.shift();
    });
    this.chart.update('none');
  }

  clear(): void {
    (this.chart.data.labels as string[]).length = 0;
    this.chart.data.datasets.forEach((dataset) => {
      dataset.data.length = 0;
    });
    this.chart.update('none');
  }
}

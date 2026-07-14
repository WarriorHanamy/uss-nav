export const CFG = {
  mqttHost: process.env.MQTT_HOST || "localhost",
  mqttPort: Number(process.env.MQTT_PORT) || 1883,
  serverPort: Number(process.env.SERVER_PORT) || 3000,
  testResultDir: "_site/test-results",
  maxContainers: 4,
  defaultDuration: 120,
  topicPrefix: "test",
} as const;

export const LLM_CONFIG = {
  model: "deepseek-chat",
  apiKey: process.env.DEEPSEEK_API_KEY || "",
  baseURL: "https://api.deepseek.com/v1",
  temperature: 0.3,
};

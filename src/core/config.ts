export const CFG = {
  mqttHost: process.env.MQTT_HOST || "localhost",
  mqttPort: Number(process.env.MQTT_PORT) || 1883,
  serverPort: Number(process.env.SERVER_PORT) || 3000,
  dockerImage: "ego-planner-test",
  dockerBaseImage: "ego-planner-sim",
  testResultDir: "_site/test-results",
  maxContainers: 8,
  defaultDuration: 300,
  topicPrefix: "test",
} as const;

export const TEST_SCENARIOS_YAML = "test-scenarios.yaml";

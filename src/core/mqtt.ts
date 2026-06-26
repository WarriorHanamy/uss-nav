import mqtt from "mqtt";
import { CFG } from "./config";

export function connectMqttClient(): mqtt.MqttClient {
  const url = `mqtt://${CFG.mqttHost}:${CFG.mqttPort}`;
  const client = mqtt.connect(url, { reconnectPeriod: 3000 });
  client.on("connect", () => console.log(`[mqtt] connected to ${url}`));
  client.on("error", (err) => console.warn(`[mqtt] error: ${err.message}`));
  return client;
}

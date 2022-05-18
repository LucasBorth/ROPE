#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
 
const char* ssid = "The Borth's";
const char* password = "senha1234ABC";

int ledFrente = D2;
int ledTras = D3;
int ledEsquerda = D4;
int ledDireita = D5;

WiFiServer server(80);

void setup() {
  Serial.begin(9600);
  Serial.println("Serial iniciado...");

  delay(500);
 
  pinMode(ledFrente, OUTPUT);
  pinMode(ledTras, OUTPUT);
  pinMode(ledEsquerda, OUTPUT);
  pinMode(ledDireita, OUTPUT);
 
  // Conecta a sua Rede WIFI e Mostra na Porta Serial
  Serial.print("Conectando a rede ");
  Serial.println(ssid);
 
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi conectado");
 
  // Inicia o Servidor
  server.begin();
  Serial.println("Servidor Iniciado");
 
  // Retorna o Valor do IP que estará nosso servidor na Rede.
  Serial.print("IP: ");
  Serial.print(WiFi.localIP());
}

void loop() {
  // Monitora se existe Cliente( alguém) usando o navegador. Pode ser acompanhado pela porta COM
  WiFiClient client = server.available();
  
  if (!client) {
    return;
  }
 
  // Servidor fica em estado de espera para alguém enviar o comando( Dados)
  Serial.println("");
  Serial.println("Cliente Online na Página!");
  
  while(!client.available()){
    delay(1);
  }
  
  // AQUI COMEÇA O CÓDIGO HTML junto com a chamada HTTP>>>>>>
 
  //  Se o Servidor, conseguiu entender a chamada que fizemos acima, Retorna o Valor Lido e mostra no Navegador. 
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println(""); // Deixei em branco para quem adivinhar o que faz aqui. Não afeta o código, mas gera algo interessante (:
    client.println("<!DOCTYPE HTML>");
    client.println("<html>");
   
    client.println("<a href='/DIRECAO=FRENTE'>Frente</a><br>");
    client.println("<a href='/DIRECAO=TRAS'>Tras</a> <br>");
    client.println("<a href='/DIRECAO=ESQUERDA'>Esquerda</a> <br>");
    client.println("<a href='/DIRECAO=DIREITA'>Direita</a><br>");
    client.println("</html>");
   
    delay(1);
    Serial.println("Cliente saiu (: ");
    Serial.println("");
  
  // Faz a leitura da chamada HTTP e mostra os valores baseado no programa acima no VOID SETUP. 
  // Lembrando que aqui não é o programa que subimos( void setup()) e sim, a parte HTML que será enviado para o servidor.
  
  String request = client.readStringUntil('\r');
  Serial.println(request);
  client.flush();
 
  // Compara o pedido de Chamada HTTP e verifica se bate com o programa que fizemos.
 
  if (request.indexOf("/DIRECAO=FRENTE") != -1) {
    digitalWrite(ledFrente, HIGH);
    digitalWrite(ledTras, LOW);
    digitalWrite(ledEsquerda, LOW);
    digitalWrite(ledDireita, LOW);
  }
  if (request.indexOf("/DIRECAO=TRAS") != -1){
    digitalWrite(ledFrente, LOW);
    digitalWrite(ledTras, HIGH);
    digitalWrite(ledEsquerda, LOW);
    digitalWrite(ledDireita, LOW);
  }
  if (request.indexOf("/DIRECAO=ESQUERDA") != -1){
    digitalWrite(ledFrente, LOW);
    digitalWrite(ledTras, LOW);
    digitalWrite(ledEsquerda, HIGH);
    digitalWrite(ledDireita, LOW);
  }
  if (request.indexOf("/DIRECAO=DIREITA") != -1){
    digitalWrite(ledFrente, LOW);
    digitalWrite(ledTras, LOW);
    digitalWrite(ledEsquerda, LOW);
    digitalWrite(ledDireita, HIGH);
  }
}

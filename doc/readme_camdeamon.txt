
JAR camdeamon

args[0] : creationTrigger json file path
args[1] : websocket server URI
-overrideTimestamp : override json timestamp value and start simulation from current time.

return codes : 
0 : normal ending never happens since main loop ends with connection interruption.
1 : argument missing, arguments are json file path and server URI, in this order
2 : exception in json file opening
3 : json synthax exception
4 : URI synthax exception
5 : thread interruption exception
6 : exception in message sending
7 : connection timeout
8 : session closed
9 : end of trajectory reached

Command line ex: 
java -jar camdeamon-1.0.1.jar "G:\Workspace\camdeamon\resources\input.json" "ws://localhost:8025/websocket/echo" [-overrideTimestamp]



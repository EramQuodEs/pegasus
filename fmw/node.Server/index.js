require('log-timestamp');
const Websocket = require("ws");

var mysql = require('mysql');
const { connect } = require("net");
const { Console } = require('console');
const { request } = require('http');
const wsserver = new Websocket.Server({clientTracking: true, port : 8989});



wsserver.on("connection", function connection(ws, req) {
  var dbcon = mysql.createConnection({
    host: "localhost",
    user: "pegasus",
    password: "$0lu$cr3@s96",
    database: "pegasus"
  });
        ws.on('error', console.error);
        const ip = req.socket.remoteAddress;
        console.log("New client connected at "+ip);
        ws.send("Welcome "+wsserver.clients[0]);
        ws.on("message", function incoming(incomer)  {
        var data = incomer.toString();
        //console.log(data);
        var res = data.charAt(0); 
        //console.log(res);
        if (res==':') {
          console.log(data);
        }
        
            if (res=='D') {
             var TS=data.substring(0,data.indexOf("#"));
             //console.log(TS) 
             //console.log("*");    
             //ws.send("#");
             var dmx=data.split('\r\n'); 
             
            
           for (var cc = 0; cc < dmx.length; cc++) {
            if (dmx[cc].length>5){
              //data VID,DID,Date,Time,DataSource,DataColumns
              
              //console.log(dmx[cc]);//store data to mysql and send back verification length
              
              //parse the data
              TS=dmx[cc].substring(dmx[cc].indexOf("#")+1,dmx[cc].length);
              //console.log(TS);
              var bmx=TS.split('\t');
              
            // console.log(bmx[4]+" " +bmx.length);
              var sql = "";
              if (bmx[4]=="IMU"/*&&bmx.length==21*/) {
                 sql = 
                "INSERT INTO IMU (vid,did,date,q1,q2,q3,q4,ax,ay,az,gx,gy,gz,mx,my,mz,pressure,temperature,altitude) VALUES ('"
                +bmx[1]+"','"+bmx[2]+"','"+bmx[3]+"','"+bmx[5]+"','"+bmx[6]+"','"+bmx[7]+"','"+bmx[8]+"','"+bmx[9]+"','"
                +bmx[10]+"','"+bmx[11]+"','"+bmx[12]+"','"+bmx[13]+"','"+bmx[14]+"','"+bmx[15]+"','"+bmx[16]+"','"+bmx[17]+"','"+bmx[18]+"','"+bmx[19]+"','"+bmx[20]
                +"')";
              }
              if (bmx[4]=="GPS"/*&&bmx.length==10*/) {

                sql = 
                "INSERT INTO GPS (vid,did,date,x,y,alt,dir,sog) VALUES ('"
                +bmx[1]+"','"+bmx[2]+"','"+bmx[3]+"','"+bmx[5]+"','"+bmx[6]+"','"+bmx[7]+"','"+bmx[8]+"','"+bmx[9]
                +"')";
              }
              if (bmx[4]=="CAN"/*&&bmx.length==10*/) {
                //	001	001	1603045665.279	CAN	1	0	0x1FFFF532	8	8E76C62D22BC0286

                sql = 
                "INSERT INTO CAN (vid,did,date,Type,RTR,SRC,MSGID,TRG,DLC,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8) VALUES ('"
                +bmx[1]+"','"//vid
                +bmx[2]+"','"//did
                +bmx[3]+"','"//date
                +bmx[5]+"','"//type
                +bmx[6]+"','"//RTR
                +parseInt(bmx[7].substring(2,4),16)+"','"//SRC
                +parseInt(bmx[7].substring(4,8),16)+"','"//MSGID//spn
                +parseInt(bmx[7].substring(8,10),16)+"','"//Target
                +bmx[8]+"','"//DLC
                +parseInt(bmx[9].substring(0,2),16)+"','"//Q1
                +parseInt(bmx[9].substring(2,4),16)+"','"
                +parseInt(bmx[9].substring(4,6),16)+"','"
                +parseInt(bmx[9].substring(6,8),16)+"','"
                +parseInt(bmx[9].substring(8,10),16)+"','"
                +parseInt(bmx[9].substring(10,12),16)+"','"
                +parseInt(bmx[9].substring(12,14),16)+"','"
                +parseInt(bmx[9].substring(14,16),16)//Q8
                +"')";
              }
              if (bmx[4]=="TIM"&&bmx.length==5) {
                sql = "INSERT INTO TIM (vid,did,date,tim) VALUES ('"
                +ser
                +"')";
              }
               if (sql.length>0){
                //console.log(sql);
                dbcon.query(sql, function (err, result) {
                  if (err) {
                    console.log("Error in 1 record"+err);
                    throw err;                  
                  }
                  //console.log(bmx[3]);
                });
              }
            }}
          }
            if (res=='L') {//log  VID,DID,Date,Time,DataSource,DataColumns
              console.log(data);//store log to mysql 
              ws.send("Log Frame==-- >>"+data.length+"<< --==");

            }
            if (res=='C') {//chat To be added later
              console.log(data);
              ws.send("Chat Frame==-- >>"+data.length+"<< --==");

            }
            if (res=='R') {//Broadcast request. [Web page >> Arduino] This is received from web page and transmitted to OBDH to set a CAN request.
              console.log("Web request....");
              wsserver.clients.forEach(function each(client) {
                if (client !== ws && client.readyState === 1) {
                  client.send(data);
                }
              });            
              
              console.log("--------"+data);
              ws.send("R"+data);

            }  
            if (res=='A') {//Broadcast answer. [Arduino >> Webpage] This is received from OBDH and sent to WebPage.
              console.log(data);
              wsserver.clients.forEach(function each(client) {
                console.log(client.ip);
                if (client !== ws && client.readyState === 1) {
                  client.send(data);
                }
              }); 
              console.log(">>>>>to web page");
              ws.send("ARoger "+data);
            }    
            if (res=='Z') {//Control frame. Contains the data to validate boradcasts.
              console.log(data);
              ws.send(data);
            }       


                    //1. Check data if ok save to mysql and send ack signal

                    //2. If not ok check length and save to seconadary table/ Send nack
                    
                    //

/*

                    
                    dbcon.connect(function(err) {
                      if (err) throw err;
                      console.log("Connected!");
                      var sql = "INSERT INTO GPS (x,y,alt) VALUES ('10','10','11')";
                      dbcon.query(sql, function (err, result) {
                        if (err) throw err;
                        console.log("1 record inserted");
                        dbcon.destroy();
                      });
                    });



*/
                   
            
        });
        ws.on("close", ()=>{
            dbcon.destroy();
            console.log("Client disconnected");
        });
    });



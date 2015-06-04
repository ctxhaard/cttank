#!/usr/bin/env node --harmony

"use strict"

const btSerial = new (require('bluetooth-serial-port')).BluetoothSerialPort(),
    targetBtName = 'MAX MERDA';

const express = require('express'),  
    app = express(),
    http = require('http').Server(app),
    io = require('socket.io')(http);
    
var strBuffer = '';

app.use(express.static('public/images/'));
app.use(express.static('public/javascripts/'));
app.use(express.static('views/'));

http.listen(3000,function(){
    console.log('listening on *:3000');
});
    
io.on('connection',function(socket){
    console.log('a user connected');
    
    console.log('inquiring...')
    btSerial.inquire();
    
    socket.on('disconnect',function(){
        console.log('user disconnected');
        // TODO: disconnect Bt
    });
});

var bufferize = function(bytes){
    var strBytes = bytes.toString('utf-8');
    console.log(strBytes);
    // enqueue buffer as string
    strBuffer += strBytes;
    //      parse buffered string
    parseBuffer();
}

var parseBuffer = function(){
    const PACKET_BORDER = "\r\n";
    const NOT_FOUND = -1; 
    const from = strBuffer.indexOf(PACKET_BORDER,0);
    if(from != NOT_FOUND)
    {
        const to = strBuffer.indexOf(PACKET_BORDER,from+PACKET_BORDER.length);
        if(to != NOT_FOUND)
        {
            const packet = strBuffer.slice(from+PACKET_BORDER.length,to);
            parsePacket(packet);
            strBuffer = strBuffer.slice(to+PACKET_BORDER.length)
        }
    }
}

var parsePacket = function(strPacket){
    //      emit events when parsing succeedes
    //      forward as heading, acceleration, ecc
    if(0 === strPacket.indexOf("H:")){
        io.emit('heading',strPacket.slice(2));
    }    
}

btSerial.on('found',function(address,name){
    if(name === targetBtName){
        btSerial.findSerialPortChannel(address,function(channel){
            btSerial.connect(address,channel,function(){
                console.log('connected');
                
                btSerial.on('data',bufferize);
            }
            ,function(){
                console.log('cannot connect');
            });
            //btSerial.close();
        },function(){
            console.log('found nothing');
        });
    }
});


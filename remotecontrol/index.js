#!/usr/bin/env node --harmony

"use strict"

// TODO: create a WebSocket server
// - accept a connection
// - connect to Bt device
// - pipe input from Rfcomm to WebSocket
const btSerial = new (require('bluetooth-serial-port')).BluetoothSerialPort(),
    targetBtName = 'MAX MERDA';

const app = require('express')(),
    http = require('http').Server(app),
    io = require('socket.io')(http);
    
app.get('/',function(req,res){
    res.sendFile('index.html',{root: '.'});
});

app.get('/cartop.png',function(req,res){
    res.sendFile('cartop.png',{root: '.'});
});

app.get('/jquery.rotate.1-1.js',function(req,res){
    res.sendFile('jquery.rotate.1-1.js',{root: '.'});
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

http.listen(3000,function(){
    console.log('listening on *:3000');
});

btSerial.on('found',function(address,name){
    if(name === targetBtName){
        btSerial.findSerialPortChannel(address,function(channel){
            btSerial.connect(address,channel,function(){
                console.log('connected');
                
                btSerial.on('data',function(buffer){
                    console.log(buffer.toString('utf-8'));
                    io.emit('msg',buffer.toString('utf-8'));
                });
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


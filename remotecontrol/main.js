/**
 * New node file
 */
var btSerial = new (require('bluetooth-serial-port')).BluetoothSerialPort();

btSerial.on('found',function(address,name){
    btSerial.findSerialPortChannel(address,function(channel){
        btSerial.connect(address,channel,function(){
            console.log('connected');
            
            btSerial.on('data',function(buffer){
                console.log(buffer.toString('utf-8'));
            });
        }
        ,function(){
            console.log('cannot connect');
        });
        //btSerial.close();
    },function(){
        console.log('found nothing');
    });
});

console.log('inquiring...')
btSerial.inquire();
console.log('...end!')

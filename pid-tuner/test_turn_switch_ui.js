// Run the actual frontend functions with DOM/fetch doubles; no car commands.
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
const path=require('node:path');
const elements=new Map();
function element(id){if(!elements.has(id))elements.set(id,{disabled:false,textContent:'',attributes:{},dataset:{},value:'',classes:new Set(),setAttribute(k,v){this.attributes[k]=v},removeAttribute(k){delete this.attributes[k]},classList:{toggle(k,v){if(v)elements.get(id).classes.add(k);else elements.get(id).classes.delete(k)}}});return elements.get(id)}
const calls=[];
const context=vm.createContext({document:{getElementById:element},fetch:async(url,options)=>{calls.push({url,body:JSON.parse(options.body)});return{ok:true,json:async()=>({ok:true})}},setTimeout,clearTimeout,console});
let code=fs.readFileSync(path.join(__dirname,'static/app.js'),'utf8');
code=code.slice(0,code.lastIndexOf('init().catch'));
vm.runInContext(code,context);
function state(data={}){context.testState=Object.assign({connected:true,params:{},commands:[],query:null,last_frame_age:0},data);vm.runInContext('state=testState;meta={token:"test"};turnSwitchBusy=false;renderTurnSwitch();',context)}
(async()=>{
 state();assert.equal(element('turn-toggle').disabled,false);assert.equal(element('turn-toggle').attributes.role,'button');assert.equal(element('turn-toggle-label').textContent,'读取开关状态');
 await vm.runInContext('toggleTurnTable()',context);assert.deepEqual(calls.map(c=>c.url),['/api/query']);assert.equal(element('turn-toggle').disabled,true);assert.equal(element('turn-toggle-label').textContent,'正在读取开关…');
 state({query:{status:'timeout'}});assert.equal(element('turn-toggle').disabled,false);assert.match(element('table-status').textContent,/5 秒内没有回复/);assert.equal(element('turn-toggle-label').textContent,'重试读取开关');
 state({query:{status:'complete'}});assert.equal(element('turn-toggle').disabled,false);assert.match(element('table-status').textContent,/本次未读到/);
 state({connected:false});assert.equal(element('turn-toggle').disabled,true);assert.match(element('table-status').textContent,/连接串口/);
 state({params:{turn_enable:{value:0}},query:{status:'complete'}});assert.equal(element('turn-toggle').disabled,false);assert.equal(element('turn-toggle').attributes.role,'switch');assert.equal(element('turn-toggle').attributes['aria-checked'],'false');
 await vm.runInContext('toggleTurnTable()',context);assert.equal(calls.at(-1).url,'/api/turn-table/switch');assert.equal(calls.at(-1).body.enabled,1);assert.equal(element('turn-toggle').attributes['aria-checked'],'false');
 state({params:{turn_enable:{value:1}}});assert.equal(element('turn-toggle').attributes['aria-checked'],'true');await vm.runInContext('toggleTurnTable()',context);assert.equal(calls.at(-1).body.enabled,0);
 state({params:{turn_enable:{value:0}},commands:[{name:'turn_enable',status:'waiting',value:1}]});assert.equal(element('turn-toggle').disabled,true);assert.equal(element('turn-toggle').attributes['aria-checked'],'false');assert.equal(element('turn-toggle-label').textContent,'等待车端确认…');
 context.testState={connected:true,params:{base_speed:{value:40,time:2}},commands:[{name:'base_speed',value:30,time:1,status:'confirmed'}],query:null,frame_count:0,rate:0,rx_bytes:0,last_frame_age:null,latest:null};
 vm.runInContext('state=testState;meta={token:"test",params:[{name:"base_speed"}],turn_table:{slots:20}};renderStatus();',context);
 assert.equal(element('s-base_speed').textContent,'车端当前值 40');
 console.log('PASS: unknown state reads only, pending states, timeout retry, disconnect, confirmed on/off and wire commands.');
})().catch(error=>{console.error(error);process.exitCode=1});

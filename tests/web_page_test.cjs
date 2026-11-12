/* Copyright 2026 EdgeZeron AI AB. See LICENSE. */
// Execute the firmware's actual page script against gateway status changes.
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const vm=require('node:vm');
const header=fs.readFileSync(path.join(__dirname,'../main/web_page.h'),'utf8');
const html=[...header.matchAll(/^\s*("(?:\\.|[^"\\])*")/gm)].map(m=>JSON.parse(m[1])).join('');
assert(html.includes('Gateway page ready.'));
const script=html.split('<script>')[1].split('</script>')[0];
const elements=new Map();
const element=id=>{
    if(!elements.has(id)) elements.set(id,{hidden:id==='photo',value:id==='resolution'?'640x480':0});
    return elements.get(id);
};
let status={radio:'starting',peer:false,state:'idle',busy:false,received:0,total:0,retries:0,image_ready:false};
let nextPoll,requests=[],blobCount=0,networkFailed=false;
const context={
    document:{getElementById:element},
    setTimeout:callback=>nextPoll=callback,
    URL:{createObjectURL:()=>`blob:test-${++blobCount}`,revokeObjectURL:()=>{}},
    fetch:async(url,options)=>{
        requests.push({url,options});
        if(networkFailed) throw Error('Network unavailable');
        return {ok:true,json:async()=>({...status}),blob:async()=>({type:'image/jpeg'})};
    }
};
vm.createContext(context);vm.runInContext(script,context);
async function test() {
    await new Promise(resolve=>setImmediate(resolve));
    assert.match(element('status').textContent,/Gateway ready/);
    assert.equal(element('capture').disabled,true);
    status={...status,radio:'ready'};await nextPoll();
    assert.match(element('connection').textContent,/No remote node/);
    assert.match(element('status').textContent,/Gateway ready/);
    assert.equal(element('capture').disabled,false); // Allows discovery on request.
    assert.equal(blobCount,0);
    await element('capture').onclick();
    const request=requests.find(r=>r.url==='/api/capture');
    assert.deepEqual(JSON.parse(request.options.body),{width:640,height:480});
    status={...status,state:'error',error:'Remote camera did not respond.'};await nextPoll();
    assert.equal(element('status').textContent,status.error);
    assert.equal(element('capture').disabled,false); // Missing camera cannot lock out retry.
    status={...status,radio:'unavailable',error:'Remote link unavailable.'};await nextPoll();
    assert.match(element('connection').textContent,/unavailable/);
    assert.equal(element('capture').disabled,true);
    status={...status,radio:'ready',peer:true,state:'complete',total:7910,received:7910,
        image_ready:true,image_id:42,test_image:true};await nextPoll();
    assert.equal(element('photo').hidden,false);assert.equal(blobCount,1);
    assert.match(element('transfer-mode').textContent,/no camera capture/);
    status={...status,state:'led-complete',total:0,received:0};await nextPoll();
    assert.equal(blobCount,1);assert.equal(element('photo').src,'blob:test-1');
    networkFailed=true;await nextPoll();
    assert.match(element('status').textContent,/Network unavailable/);
    assert.equal(element('photo').hidden,false);
    networkFailed=false;await nextPoll();
    assert.match(element('status').textContent,/LED acknowledged/);
    console.log('PASS: page survives missing peer, link startup/failure, capture failure and reconnect; image download and resolution remain functional');
}
test().catch(error=>{console.error(error);process.exitCode=1;});

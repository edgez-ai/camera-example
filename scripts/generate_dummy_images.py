#!/usr/bin/env python3
"""Regenerate deterministic transport fixtures (developer-only Pillow dependency)."""
from pathlib import Path
import hashlib
import json
import zlib
from PIL import Image,ImageDraw,ImageFont
root=Path(__file__).resolve().parents[1]
assets=root/'testdata/dummy_images';assets.mkdir(parents=True,exist_ok=True)
modes=[(160,120),(320,240),(640,480),(1024,768),(1280,720),(1280,960),(1600,1200),(1920,1080),(2304,1296)]
source=['/* Copyright 2026 EdgeZeron AI AB. See LICENSE. Generated test fixtures. */',
        '#include "camera_dummy.h"']
entries=[]
manifest=[]
for width,height in modes:
    image=Image.new('RGB',(width,height),'#edf1e9');draw=ImageDraw.Draw(image)
    colors=['#dc4654','#e4bb45','#4b9a68','#4c9eb1','#526da5','#a45b95']
    for i,color in enumerate(colors):
        draw.rectangle((i*width//6,0,(i+1)*width//6,height),fill=color)
    draw.rectangle((0,height//3,width,height*2//3),fill='#17372e')
    font=ImageFont.load_default(size=max(10,width//28))
    text='TEST IMAGE\nNO CAMERA\n%d x %d'%(width,height)
    box=draw.multiline_textbbox((0,0),text,font=font,spacing=4,align='center')
    draw.multiline_text(((width-(box[2]-box[0]))//2,(height-(box[3]-box[1]))//2),
        text,font=font,fill='white',spacing=4,align='center')
    path=assets/('%dx%d.jpg'%(width,height))
    image.save(path,'JPEG',quality=60,optimize=True,subsampling=2,progressive=False)
    data=path.read_bytes();name='image_%d_%d'%(width,height);crc=zlib.crc32(data)
    source.append('static const uint8_t %s[] = {'%name)
    for at in range(0,len(data),16):source.append('    '+','.join('0x%02x'%b for b in data[at:at+16])+',')
    source.append('};')
    entries.append('    {%d,%d,sizeof(%s),0x%08xU,%s},'%(width,height,name,crc,name))
    manifest.append(dict(width=width,height=height,bytes=len(data),crc32='%08x'%crc,sha256=hashlib.sha256(data).hexdigest()))
source.extend(['static const struct cam_dummy_image images[] = {',*entries,'};',
    'const struct cam_dummy_image *cam_dummy_find(uint16_t width,uint16_t height) {',
    '    for(size_t i=0;i<sizeof(images)/sizeof(images[0]);i++)',
    '        if(images[i].width==width && images[i].height==height) return &images[i];',
    '    return NULL;', '}'])
(root/'nrf54/src/camera_dummy_images.c').write_text('\n'.join(source)+'\n')
(assets/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print('Generated %d images, %d JPEG bytes'%(len(manifest),sum(x['bytes'] for x in manifest)))

#!/usr/bin/env python3
"""hull_plan.png / hull_topmap.png (두 배가 위아래로 한 장) 를 배별로 자른다 → *_A.png (1번배, 위) / *_B.png (2번배, 아래).
원본 생성기는 저장소 밖(STL 기반)이라 자르기만 한다. 재현: python3 tools/split_hull_figs.py"""
import os
from PIL import Image
D=os.path.join(os.path.dirname(os.path.abspath(__file__)),"..","docs","전달용","배치도")
for name,split in (("hull_plan",540),("hull_topmap",495)):
    im=Image.open(os.path.join(D,name+".png")); w,h=im.size
    im.crop((0,0,w,split)).save(os.path.join(D,name+"_A.png"))
    im.crop((0,split,w,h)).save(os.path.join(D,name+"_B.png"))
    print(name,w,h,"→ A",split,"/ B",h-split)

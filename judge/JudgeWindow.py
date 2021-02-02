#!/usr/bin/env python
# -*- coding: utf8 -*-

# Dependencies : sudo apt install python-pil.imagetk

import Tkinter as tk
from PIL import Image,ImageTk
import os
import time
import copy
import JudgeReceiver

class JudgeWindow:
    def __init__(self,window_name="burger war",window_size=(520,400)):
        self.script_dir = os.path.dirname(os.path.abspath(__file__))

        #ジャッジサーバー通信クラス
        self.judge = JudgeReceiver.JudgeReceiver()

        # ウィンドウ作成
        self.root = tk.Tk()
        self.root.title(window_name)
        self.root.minsize(window_size[0], window_size[1])
        self.window_size=window_size

        self.canvas = tk.Canvas(bg="black", width=window_size[0], height=window_size[1]) 
        self.canvas.pack(expand=True,fill=tk.BOTH)

        self.canvas.bind("<Configure>", self.on_resize)

        self.read_images()
        self.draw()


        self.update_status()

        self.root.after(1000,self.update_status)

    def on_resize(self,event):
        self.window_size=(event.width,event.height)
        self.draw()
        
    def update_status(self):
        self.judge.getStatus()

        self.canvas.itemconfig("state",text="Game State: "+self.judge.state)

        self.canvas.itemconfig("time",text=int(self.judge.time))
        self.canvas.itemconfig("team_blue",text=self.judge.team_blue)
        self.canvas.itemconfig("team_red",text=self.judge.team_red)

        if self.judge.team_blue_ready:
            self.canvas.itemconfig("team_blue",fill="blue")
        else:
            self.canvas.itemconfig("team_blue",fill="gray")

        if self.judge.team_red_ready:
            self.canvas.itemconfig("team_red",fill="red")
        else:
            self.canvas.itemconfig("team_red",fill="gray")

        self.canvas.itemconfig("point_blue",text=self.judge.point_blue)
        self.canvas.itemconfig("point_red",text=self.judge.point_red)

        if len(self.judge.targets)>0:
            for target in self.judge.targets:
                if target["player"]=="n":
                    self.canvas.itemconfig(target["name"],state=tk.HIDDEN)
                elif target["player"]=="b":
                    self.canvas.itemconfig(target["name"],state=tk.NORMAL)
                    self.canvas.itemconfig(target["name"],fill='blue')
                elif target["player"]=="r":
                    self.canvas.itemconfig(target["name"],state=tk.NORMAL)
                    self.canvas.itemconfig(target["name"],fill='red')

        self.root.after(1000,self.update_status)


    def read_images(self):
        self.img_org = Image.open(open(self.script_dir + '/picture/field_burger.png','rb'))

    def draw(self):
        self.canvas.delete("all")

        img_org=copy.deepcopy(self.img_org)
        img_org.thumbnail(self.window_size,Image.ANTIALIAS)
        self.img = ImageTk.PhotoImage(img_org)

        #背景画像
        self.canvas.create_image(0, 0, image=self.img, anchor=tk.NW)

        #ゲームステート
        self.state=self.canvas.create_text(
            0.5 * self.img.width(),
            0.03 * self.img.height(),
            text="Game State: end",font=("",int(0.05*self.img.height())),
            fill="black",tag='state')

        #チーム名表示と点数
        self.team_blue=self.canvas.create_text(
            0.2 * self.img.width(),
            0.09 * self.img.height(),
            text="blue",font=("",int(0.06*self.img.height())),
            fill="blue",tag='team_blue')
        self.team_red=self.canvas.create_text(
            0.2*4*self.img.width(),
            0.09*self.img.height(),
            text="red",font=("",int(0.06*self.img.height(),)),
            fill="red",tag='team_red')
        self.point_blue=self.canvas.create_text(
            0.2*self.img.width(),
            0.09*2*self.img.height(),
            text="0",font=("",int(0.06*self.img.height(),)),
            fill="blue",tag='point_blue')
        self.point_red=self.canvas.create_text(
            0.2*4*self.img.width(),
            0.09*2*self.img.height(),
            text="0",font=("",int(0.06*self.img.height(),)),
            fill="red",tag="point_red")

        #時間
        self.time=self.canvas.create_text(
            0.5*self.img.width(),
            0.25*self.img.height(),
            text="0000",font=("",int(0.05*self.img.height(),)),
            fill="black",tag="time")

        #中央BOX
        self.FriedShrimp_N=self.canvas.create_rectangle(
            (0.51-0.04)*self.img.width() , (0.59-0.01)*self.img.height(),
            (0.51+0.04)*self.img.width() , (0.59+0.01)*self.img.height(),
            fill="red",
            width=0,
            tag="FriedShrimp_N"
        )
        self.FriedShrimp_S=self.canvas.create_rectangle(
            (0.51-0.04)*self.img.width() , (0.73-0.01)*self.img.height(),
            (0.51+0.04)*self.img.width() , (0.73+0.01)*self.img.height(),
            fill="red",
            width=0,
            tag="FriedShrimp_S"
        )
        self.FriedShrimp_W=self.canvas.create_rectangle(
            (0.46-0.007)*self.img.width() , (0.66-0.05)*self.img.height(),
            (0.46+0.007)*self.img.width() , (0.66+0.05)*self.img.height(),
            fill="red",
            width=0,
            tag="FriedShrimp_W"
        )
        self.FriedShrimp_E=self.canvas.create_rectangle(
            (0.56-0.007)*self.img.width() , (0.66-0.05)*self.img.height(),
            (0.56+0.007)*self.img.width() , (0.66+0.05)*self.img.height(),
            fill="red",
            width=0,
            tag="FriedShrimp_E"
        )
        #左上
        self.Tomato_N=self.canvas.create_rectangle(
            (0.4-0.04)*self.img.width() , (0.49-0.01)*self.img.height(),
            (0.4+0.04)*self.img.width() , (0.49+0.01)*self.img.height(),
            fill="red",
            width=0,
            tag="Tomato_N"
        )
        self.Tomato_S=self.canvas.create_rectangle(
            (0.4-0.04)*self.img.width() , (0.6-0.01)*self.img.height(),
            (0.4+0.04)*self.img.width() , (0.6+0.01)*self.img.height(),
            fill="red",
            width=0,
            tag="Tomato_S"
        )
        #右上
        self.Omelette_N=self.canvas.create_rectangle(
            (0.62-0.04)*self.img.width() , (0.49-0.01)*self.img.height(),
            (0.62+0.04)*self.img.width() , (0.49+0.01)*self.img.height(),
            fill="red",
            width=0,
            tag="Omelette_N"
        )
        self.Omelette_N=self.canvas.create_rectangle(
            (0.62-0.04)*self.img.width() , (0.6-0.01)*self.img.height(),
            (0.62+0.04)*self.img.width() , (0.6+0.01)*self.img.height(),
            fill="red",
            width=0,
            tag="Omelette_N"
        )
        #左下
        self.Pudding_N=self.canvas.create_rectangle(
            (0.4-0.04)*self.img.width() , (0.71-0.01)*self.img.height(),
            (0.4+0.04)*self.img.width() , (0.71+0.01)*self.img.height(),
            fill="red",
            width=0,
            tag="Pudding_N"
        )
        self.Pudding_S=self.canvas.create_rectangle(
            (0.4-0.04)*self.img.width() , (0.82-0.01)*self.img.height(),
            (0.4+0.04)*self.img.width() , (0.82+0.01)*self.img.height(),
            fill="red",
            width=0,
            tag="Pudding_S"
        )
        #右下
        self.OctopusWiener_N=self.canvas.create_rectangle(
            (0.62-0.04)*self.img.width() , (0.71-0.01)*self.img.height(),
            (0.62+0.04)*self.img.width() , (0.71+0.01)*self.img.height(),
            fill="red",
            width=0,
            tag="OctopusWiener_N"
        )
        self.OctopusWiener_S=self.canvas.create_rectangle(
            (0.62-0.04)*self.img.width() , (0.82-0.01)*self.img.height(),
            (0.62+0.04)*self.img.width() , (0.82+0.01)*self.img.height(),
            fill="red",
            width=0,
            tag="OctopusWiener_S"
        )

        #ロボット青
        self.BL_B=self.canvas.create_rectangle(
            (0.285-0.04)*self.img.width() , (0.3-0.01)*self.img.height(),
            (0.285+0.04)*self.img.width() , (0.3+0.01)*self.img.height(),
            fill="red",
            width=0,
            tag="BL_B"
        )
        self.BL_R=self.canvas.create_rectangle(
            (0.2-0.007)*self.img.width() , (0.37-0.05)*self.img.height(),
            (0.2+0.007)*self.img.width() , (0.37+0.05)*self.img.height(),
            fill="red",
            width=0,
            tag="BL_R"
        )
        self.BL_L=self.canvas.create_rectangle(
            (0.37-0.007)*self.img.width() , (0.37-0.05)*self.img.height(),
            (0.37+0.007)*self.img.width() , (0.37+0.05)*self.img.height(),
            fill="red",
            width=0,
            tag="BL_L"
        )
        #ロボット赤
        self.RE_B=self.canvas.create_rectangle(
            (0.735-0.04)*self.img.width() , (0.99-0.01)*self.img.height(),
            (0.735+0.04)*self.img.width() , (0.99+0.01)*self.img.height(),
            fill="blue",
            width=0,
            tag="RE_B"
        )
        self.RE_R=self.canvas.create_rectangle(
            (0.65-0.007)*self.img.width() , (0.92-0.05)*self.img.height(),
            (0.65+0.007)*self.img.width() , (0.92+0.05)*self.img.height(),
            fill="blue",
            width=0,
            tag="RE_R"
        )
        self.RE_L=self.canvas.create_rectangle(
            (0.82-0.007)*self.img.width() , (0.92-0.05)*self.img.height(),
            (0.82+0.007)*self.img.width() , (0.92+0.05)*self.img.height(),
            fill="blue",
            width=0,
            tag="RE_L"
        )



    def mainloop(self):
        self.root.mainloop()


 
 

if __name__ == "__main__":

    window = JudgeWindow(window_size=(300,225))
    # メインループ
    window.mainloop()


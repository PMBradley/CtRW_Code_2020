package org.firstinspires.ftc.teamcode.util;


public class DuoHolder { // this is just a class to hold two objects in one, kinda like a dictionary thing, but with one unit. Good with ArrayLists
    private Object obj1;
    private Object obj2;

    public DuoHolder(){}
    public DuoHolder (Object obj1){
        this.obj1 = obj1;
    }
    public DuoHolder (Object obj1, Object obj2){
        this.obj1 = obj1;
        this.obj2 = obj2;
    }

    public Object getObj1(){return obj1;}
    public Object getObj2(){return obj2;}
    public void setObj1(Object obj1){ this.obj1 = obj1;}
    public void setObj2(Object obj2){ this.obj1 = obj2;}
}

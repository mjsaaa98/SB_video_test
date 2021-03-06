#include "armorpredict.h"

ArmorPredict::ArmorPredict(){
}

void ArmorPredict::Fresh()
{
    yaw_old = yaw_out;
    pitch_old = pitch_out;
    yaw_out = 0;
    pitch_out = 0;
}

int ArmorPredict::Predict(vector<AbsPosition> Positions){
    Fresh();
    int k=0;
    if(Positions.size() != 0){
//        if(Positions.size() != 1){
//            sort(Positions.begin(),Positions.end(),PosSort);
//        }
        if(Positions.size() > 1){
            cout<<"dis:"<<Positions[k].z<<endl;
            for(int i=0;i<Positions.size()-1;i++)
            {
                cout<<"dis(i):"<<Positions[i+1].z<<endl;
                if(Positions[k].z>Positions[i+1].z)
                    k = i+1;
            }
            Result = Positions[k];
            cout<<"min_dis:"<<Positions[k].z<<endl;
        }else{
            Result = Positions[0];
        }

        if(Result.z < 30){
            yaw_out = 0;
            pitch_out = 0;
        }else{
            AngleFit(Result);
        }
        Vision = {yaw_out,pitch_out,Result.z,0,1};
        OldPositions = Positions;
        OldResult = Result;
    }
        return k;
}

void ArmorPredict::AngleFit( AbsPosition& input){
    float flytime,gravity_offset;
    float shoot_distance = sqrtf(input.x * input.x + input.y * input.y + input.z * input.z)*0.001; // m
    shoot_speed = shoot_distance*10.0;
    int shootspeedmax = 30;
    if(shoot_speed > shootspeedmax){
        shoot_speed = shootspeedmax;
        flytime = shoot_distance/shoot_speed;
        gravity_offset = 4.905 * flytime * flytime;
    }else if(shoot_speed < 14){
        shoot_speed = 14;
        flytime = shoot_distance/shoot_speed;
        gravity_offset = 4.905 * flytime * flytime;
    }else{
        gravity_offset = 0.04905;
    }
    yaw_out = atan(input.x/input.z) * 180/3.14159265;
    pitch_out = atan((input.y-gravity_offset)/input.z) * 180/3.14159265;
    input.z = shoot_distance*1000;
}

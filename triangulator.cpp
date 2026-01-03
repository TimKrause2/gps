#include "triangulate.h"

#define c (2.99792458e8)
#define R_EARTH 6371e3

Triangulator::Triangulator(int fs)
    : fs(fs)
{
    thread = std::thread(&Triangulator::thread_func, this);
}

Triangulator::~Triangulator(void)
{
    queue.stop();
    thread.join();
}

void Triangulator::send_add_message(int sat, SatelliteFix &fix)
{
    queue.push(std::make_unique<TriangulateAddMessage>(sat, fix));
}

void Triangulator::send_del_message(int sat)
{
    queue.push(std::make_unique<TriangulateDelMessage>(sat));
}

void Triangulator::thread_func(void)
{
    while(true){
        std::unique_ptr<TriangulateMessage> tm = queue.pop();
        if(tm == nullptr)
            break;
        switch(tm->type){
        case TYPE_ADD:
        {
            TriangulateAddMessage *tam = static_cast<TriangulateAddMessage*>(tm.get());
            add_sat(tam);
        }
        break;
        case TYPE_DEL:
            TriangulateDelMessage *tdm = static_cast<TriangulateDelMessage*>(tm.get());
            del_sat(tdm);
            break;
        }
    }
}

void Triangulator::add_sat(TriangulateAddMessage *tam)
{
    // search for this sat and replace the old one or add a new sat
    std::list<TriangulateAddMessage>::iterator list_it = collected_sats.begin();
    bool found = false;
    for(;list_it!=collected_sats.end();){
        if(list_it->sat == tam->sat){
            found = true;
            // erase the old sat
            list_it = collected_sats.erase(list_it);
            // insert the new sat
            collected_sats.push_front(*tam);
            break;
        }
    }
    if(!found){
        // wasn't found so its a new sat. Add to the end of the list.
        collected_sats.push_front(*tam);
    }
    triangulate();
}

void Triangulator::del_sat(TriangulateDelMessage *tdm)
{
    std::list<TriangulateAddMessage>::iterator list_it = collected_sats.begin();
    for(;list_it!=collected_sats.end();){
        if(list_it->sat == tdm->sat){
            collected_sats.erase(list_it);
            break;
        }
    }
}

Vector4d Triangulator::jacobian_vec(int f, Vector4d &X)
{
    double pf_px = 2.0*(X[0] - fixs[f].x_k);
    double pf_py = 2.0*(X[1] - fixs[f].y_k);
    double pf_pz = 2.0*(X[2] - fixs[f].z_k);
    double tilde_t = (double)fixs[f].sample_index/fs;
    double pf_pb = 2.0*(tilde_t - X[3] - fixs[f].gps_time)*c*c;

    Vector4d vec(pf_px, pf_py, pf_pz, pf_pb);
    return vec;
}

Matrix4d Triangulator::jacobian(Vector4d &X)
{
    Matrix4d J;
    for(int f=0;f<4;f++){
        J.row(f) = jacobian_vec(f, X);
    }
    return J;
}

double Triangulator::func(int f, Vector4d &X)
{
    double dx = X[0] - fixs[f].x_k;
    double dy = X[1] - fixs[f].y_k;
    double dz = X[2] - fixs[f].z_k;
    double tilde_t = (double)fixs[f].sample_index/fs;
    double dt = (tilde_t - X[3] - fixs[f].gps_time)*c;
    return dx*dx + dy*dy + dz*dz + dt*dt;
}

Vector4d Triangulator::dY(Vector4d &X)
{
    return Vector4d(func(0,X),func(1,X),func(2,X),func(3,X));
}

Vector4d Triangulator::X_guess(void)
{
    // find the average point of the satellites
    Vector3d R(0,0,0);
    for(int f=0;f<4;f++){
        R[0] += fixs[f].x_k;
        R[1] += fixs[f].y_k;
        R[2] += fixs[f].z_k;
    }
    R /= 4;
    // interpolate to the surface of the earth (assuming a sphere)
    Vector3d N = R.normalized();
    Vector3d R_g = N*R_EARTH;
    Vector3d R_g_sat(fixs[0].x_k, fixs[0].y_k, fixs[0].z_k);
    R_g_sat -= R_g;
    Vector4d X;
    X.segment<3>(0) = R_g;
    //
    // Since there are zeros in the satellite time and gps_time it
    // is necessary to initialize the bias to some value other than
    // 0.
    //
    X[3] = -R_g_sat.norm()/c;
    return X;
}

void Triangulator::gps_coordinates(Vector4d &X)
{
    Vector3d R = X.segment<3>(0);
    double longitude = std::atan2(R[1],R[0])*180.0/M_PI;
    double latitude = std::asin(R[2]/R.norm())*180.0/M_PI;
    printf("Triangulator::gps_coordinates longitude:%lf latitude:%lf bias:%.7le\n",
           longitude, latitude, X[3]);
}

void Triangulator::triangulate(void)
{
    if(collected_sats.size()<4){
        return;
    }

    // pick the first four satellites for triangulation
    int i=0;
    for(auto &tam : collected_sats){
        fixs[i] = tam.fix;
        if(++i==4)
            break;
    }

    // validate the gps_times for the satellites
    for(i=1;i<4;i++){
        if(abs(fixs[0].gps_time - fixs[i].gps_time)>0.5){
            return;
        }
    }

    // make all times relative to the first fix
    for(int i=1;i<4;i++){
        fixs[i].gps_time     -= fixs[0].gps_time;
        fixs[i].sample_index -= fixs[0].sample_index;
    }
    fixs[0].gps_time = 0.0;
    fixs[0].sample_index = 0;

    Vector4d X = X_guess();
    while(true){
        Matrix4d J = jacobian(X);
        Vector4d delta_Y = dY(X);
        Vector4d delta_X = J.inverse()*delta_Y;
        X -= delta_X;
        if(delta_X.cwiseAbs().maxCoeff() < 1e-8)
            break;
    }

    gps_coordinates(X);
}

#include "MyKinoRRT.h"

void MyDynamicAgent::rungeKutta45(Eigen::VectorXd& state, const Eigen::VectorXd& control, double t) {
    double max_dy = 1.0;
    double dt = t*0.1;
    double t_pass = 0.0;
    while(true){
        if(t_pass + dt > t){
            dt = t - t_pass;
        }
        Eigen::VectorXd k1 = dynamic(state, control);
        Eigen::VectorXd k2 = dynamic(state + dt * (1.0/5.0) * k1, control);
        Eigen::VectorXd k3 = dynamic(state + dt * (3.0/40.0) * k1 + dt * (9.0/40.0) * k2, control);
        Eigen::VectorXd k4 = dynamic(state + dt * (44.0/45.0) * k1 - dt * (56.0/15.0) * k2 + dt * (32.0/9.0) * k3, control);
        Eigen::VectorXd k5 = dynamic(state + dt * (19372.0/6561.0) * k1 - dt * (25360.0/2187.0) * k2 + dt * (64448.0/6561.0) * k3 - dt * (212.0/729.0) * k4, control);
        Eigen::VectorXd k6 = dynamic(state + dt * (9017.0/3168.0) * k1 - dt * (355.0/33.0) * k2 + dt * (46732.0/5247.0) * k3 + dt * (49.0/176.0) * k4 - dt * (5103.0/18656.0) * k5, control);
        Eigen::VectorXd k7 = dynamic(state + dt * (35.0/384.0) * k1 + dt * (500.0/1113.0) * k3 + dt * (125.0/192.0) * k4 - dt * (2187.0/6784.0) * k5 + dt * (11.0/84.0) * k6, control);

        Eigen::VectorXd y_low = state + dt * ((35.0/384.0) * k1 + (500.0/1113.0) * k3 + (125.0/192.0) * k4 - (2187.0/6784.0) * k5 + (11.0/84.0) * k6);
        Eigen::VectorXd y_high = state + dt * ((5179.0/57600.0) * k1 + (7571.0/16695.0) * k3 + (393.0/640.0) * k4 - (92097.0/339200.0) * k5 + (187.0/2100.0) * k6 + (1.0/40.0) * k7);   
        max_dy = (y_high - y_low).cwiseAbs().maxCoeff();
        
        if(max_dy < EPSILON){
            t_pass += dt;
            state = y_high;
            if(t_pass >= t){
                return ;
            }
        }
        double scale = 0.9 * pow(EPSILON / (max_dy + 1e-10), 0.2);
        dt = dt*scale;
        if(dt < 1e-10){
            for(int i = 0 ; i < state.size(); i++) {
                state(i) = 1000000; // indicate failure
            }
            return;
        }

    }
}

Eigen::VectorXd MySingleIntegrator::dynamic(const Eigen::VectorXd& state, const Eigen::VectorXd& control) {
    return control;
}

Eigen::VectorXd MyFirstOrderUnicycle::dynamic(const Eigen::VectorXd& state, const Eigen::VectorXd& control) {
    Eigen::VectorXd dxdt(3);
    double theta = state(2);
    double v = control(0);
    double omega = control(1);
    dxdt(0) = v * cos(theta)*0.25;
    dxdt(1) = v * sin(theta)*0.25;
    dxdt(2) = omega;
    return dxdt;
}

Eigen::VectorXd MySecondOrderUnicycle::dynamic(const Eigen::VectorXd& state, const Eigen::VectorXd& control) {
    Eigen::VectorXd dxdt(5);
    double theta = state(2);
    double v = state(3);
    double omega = state(4);
    double a = control(0);
    double alpha = control(1);
    dxdt(0) = v * cos(theta)*0.25;
    dxdt(1) = v * sin(theta)*0.25;
    dxdt(2) = omega;
    dxdt(3) = a;
    dxdt(4) = alpha;
    return dxdt;
}

amp::KinoPath MyKinoRRT::get_pre_plan_path(){
    // DEBUG(temp);
    amp::KinoPath pre_plane_path;

    pre_plane_path.valid = true;

    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 0.5,5.5,0,2,0).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 0.865607,5.50139,0.0115317,1.86213,0.313961).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 1.26812,5.50832,0.0155548,1.69171,-0.225071).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 1.6374,5.51209,0.00802931,1.55227,0.0266725).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 1.93018,5.51458,0.00858508,1.63794,-0.00738811).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 2.1717,5.51736,0.0177274,1.48648,0.382927).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 2.80078,5.53244,0.012049,1.67739,-0.451442).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 2.8857,5.53314,0.00482456,1.68836,-0.352316).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 3.05314,5.53305,-0.00513762,1.73055,-0.225684).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 3.86407,5.52921,0.0144323,1.94376,0.444019).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 4.68135,5.57014,0.0824434,2.11747,0.344273).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 5.70992,5.69178,0.151128,2.07694,0.295654).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 5.71019,5.69182,0.151145,2.07695,0.295262).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 6.4253,5.80415,0.143345,2.13713,-0.395356).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 6.85852,5.86119,0.12415,1.93669,-0.0260375).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 7.45774,5.9275,0.0831471,2.08511,-0.603519).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 7.49845,5.93078,0.0777,2.08796,-0.57257).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 7.6324,5.94013,0.0623354,2.08909,-0.465715).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 8.07039,5.95447,-0.00544865,2.17173,-0.831147).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 8.1275,5.95382,-0.017053,2.17948,-0.754509).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 8.66375,5.92684,-0.0692044,2.26753,-0.121048).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 8.83749,5.91422,-0.0771195,2.24265,-0.324538).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 9.85541,5.83906,-0.0309909,2.41019,0.71559).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 10.6502,5.85996,0.0729235,2.51886,0.435136).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 11.7606,5.94136,0.0184773,2.52829,-0.845358).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 12.6353,5.8821,-0.141789,2.21841,-0.622157).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 13.3425,5.75387,-0.203909,1.87179,-0.165786).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 13.5529,5.70947,-0.212421,1.7597,-0.225529).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 13.7766,5.66034,-0.218479,1.67837,-0.035088).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 13.9173,5.62894,-0.22133,1.59499,-0.16289).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 14.2111,5.56114,-0.232665,1.41496,-0.209526).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 14.5703,5.47468,-0.235452,1.26061,0.141622).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 14.8983,5.40032,-0.201155,1.04636,0.785519).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 14.9861,5.38332,-0.180621,0.965249,0.921376).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 15.2575,5.3401,-0.145665,0.853139,0.129107).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 15.4241,5.31636,-0.135095,0.681443,0.486022).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 15.6413,5.28771,-0.135974,0.290952,-0.669578).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 15.644,5.28734,-0.136405,0.282088,-0.655999).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 15.6747,5.28306,-0.14026,0.1696,-0.433464).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 15.6871,5.2813,-0.141206,0.0817098,-0.265635).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 15.6914,5.28069,-0.141518,-0.00678573,-0.531108).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 15.6499,5.28645,-0.132807,-0.219773,-0.932704).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 15.6019,5.29255,-0.120937,-0.3161,-0.846297).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 15.58,5.29516,-0.115095,-0.376543,-0.990094).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 15.4616,5.3069,-0.0840889,-0.611887,-0.84914).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 15.4332,5.3092,-0.077803,-0.628148,-0.821436).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 15.0298,5.32701,-0.0178087,-1.01897,-0.468764).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 14.5296,5.32126,0.044696,-1.34043,-0.637381).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 14.5227,5.32094,0.0457098,-1.34261,-0.627174).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 14.0913,5.29074,0.087876,-1.62242,-0.281004).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 14.027,5.28494,0.092061,-1.62285,-0.345835).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 13.6863,5.24593,0.147825,-1.6919,-0.963353).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 12.9219,5.05224,0.336955,-1.89887,-0.783664).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 12.6636,4.95548,0.374982,-2.00809,-0.41095).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 12.0416,4.6886,0.43649,-2.0581,-0.441737).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 11.7132,4.52883,0.468109,-1.97352,-0.37447).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 11.0669,4.17957,0.52113,-1.90629,-0.31755).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 10.6237,3.91719,0.544509,-1.8027,-0.125636).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 10.6232,3.91688,0.544524,-1.80255,-0.125292).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 9.9238,3.48199,0.569554,-1.5679,-0.177455).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 9.84295,3.43008,0.571369,-1.54902,-0.0101363).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 9.28412,3.07271,0.563416,-1.44145,0.131384).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 9.01827,2.90745,0.545598,-1.3445,0.422522).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 8.76387,2.75844,0.511466,-1.25864,0.625107).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 8.34706,2.54132,0.451584,-1.23537,0.507421).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 8.29784,2.51763,0.446062,-1.22731,0.42797).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 8.04844,2.40215,0.421268,-1.15592,0.419388).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 7.96499,2.36509,0.415315,-1.15156,0.208316).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 7.66685,2.23669,0.39617,-1.16342,0.36374).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 7.50633,2.17131,0.374322,-1.14196,0.746716).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 7.21903,2.06748,0.320046,-1.1179,0.705472).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 7.04788,2.01387,0.285634,-1.12141,0.821539).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 6.93691,1.9826,0.265199,-1.12795,0.623197).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 6.83621,1.95595,0.253005,-1.08893,0.43116).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 6.62867,1.90427,0.235994,-1.00539,0.323617).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 6.38927,1.85033,0.199623,-0.958523,0.913312).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 6.17217,1.81202,0.152982,-0.948251,0.705638).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 5.81759,1.76723,0.101316,-0.855814,0.539859).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 5.76117,1.76169,0.094576,-0.825353,0.532745).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 5.43676,1.73539,0.0725518,-0.643097,0.0906146).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 5.2869,1.72469,0.070068,-0.554927,0.073916).finished());
    pre_plane_path.waypoints.push_back((Eigen::VectorXd(5) << 5.1544,1.71554,0.0678634,-0.466172,0.0922245).finished());

    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.728169,1.65826).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.752237,-2.37923).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.612422,1.10567).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.466752,-0.185562).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.979658,2.52448).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.479933,-2.09756).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.217424,1.96435).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.430718,1.29278).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.483021,1.51716).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.430963,-0.247469).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.0820357,-0.0984276).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.0704471,-2.98004).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.17515,-2.0102).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.934332,1.7216).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.495018,-1.92604).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.145484,1.58098).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.0175918,1.66207).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.401666,-1.77618).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.294995,2.91945).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.364604,2.62297).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.322063,-2.63423).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.381826,2.37045).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.336707,-0.868952).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.0213616,-2.90195).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.83796,0.603561).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.986103,1.29836).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.946503,-0.504456).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.610364,1.42917).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.946785,-1.4513).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.898483,-0.232746).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.558959,1.27161).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.734765,2.20831).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.912469,1.52826).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.370881,-2.62097).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.78279,1.62724).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.866567,-2.56448).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.919522,1.40848).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.820006,1.62223).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.879177,1.67881).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.767586,-2.30264).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.57497,-1.08413).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.534339,0.479307).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.946083,-2.25078).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.977456,0.585423).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.354501,0.603981).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.796994,0.719198).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.758009,-0.397597).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.421952,1.97909).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.959062,1.1865).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.0107489,-1.63001).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.33367,-2.98379).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.470527,0.408511).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.773582,2.63992).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.150222,-0.0924678).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.46685,0.37126).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.177503,0.150278).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.372965,0.691023).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.47159,1.0631).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.480141,-0.106734).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.306148,2.71419).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.242478,0.319006).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.431432,1.29554).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.379037,0.894283).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.0617405,-0.312221).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.181579,-1.79096).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.309498,-0.0372062).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.0551903,-2.66716).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.0423104,0.554194).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.142731,2.54687).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.0889636,-0.152538).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.02187,0.724528).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << -0.0638564,-1.93494).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.415228,-2.04345).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.409003,-0.526518).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.187531,2.35948).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.0444175,-0.898037).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.233275,-0.418361).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.45167,-0.105474).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.411138,-0.99737).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.351534,-0.0665775).finished());
    pre_plane_path.controls.push_back((Eigen::VectorXd(2) << 0.341183,0.0703796).finished());

    pre_plane_path.durations.push_back(0.189331);
    pre_plane_path.durations.push_back(0.226557);
    pre_plane_path.durations.push_back(0.227685);
    pre_plane_path.durations.push_back(0.183554);
    pre_plane_path.durations.push_back(0.154612);
    pre_plane_path.durations.push_back(0.397781);
    pre_plane_path.durations.push_back(0.0504623);
    pre_plane_path.durations.push_back(0.0979533);
    pre_plane_path.durations.push_back(0.44142);
    pre_plane_path.durations.push_back(0.403062);
    pre_plane_path.durations.push_back(0.493963);
    pre_plane_path.durations.push_back(0.000131444);
    pre_plane_path.durations.push_back(0.343556);
    pre_plane_path.durations.push_back(0.214521);
    pre_plane_path.durations.push_back(0.299828);
    pre_plane_path.durations.push_back(0.0195755);
    pre_plane_path.durations.push_back(0.0642907);
    pre_plane_path.durations.push_back(0.205741);
    pre_plane_path.durations.push_back(0.026251);
    pre_plane_path.durations.push_back(0.241505);
    pre_plane_path.durations.push_back(0.0772485);
    pre_plane_path.durations.push_back(0.438789);
    pre_plane_path.durations.push_back(0.322749);
    pre_plane_path.durations.push_back(0.441252);
    pre_plane_path.durations.push_back(0.369806);
    pre_plane_path.durations.push_back(0.351498);
    pre_plane_path.durations.push_back(0.11843);
    pre_plane_path.durations.push_back(0.133253);
    pre_plane_path.durations.push_back(0.0880606);
    pre_plane_path.durations.push_back(0.200373);
    pre_plane_path.durations.push_back(0.276144);
    pre_plane_path.durations.push_back(0.291579);
    pre_plane_path.durations.push_back(0.0888969);
    pre_plane_path.durations.push_back(0.30228);
    pre_plane_path.durations.push_back(0.219338);
    pre_plane_path.durations.push_back(0.450618);
    pre_plane_path.durations.push_back(0.00964044);
    pre_plane_path.durations.push_back(0.137179);
    pre_plane_path.durations.push_back(0.0999689);
    pre_plane_path.durations.push_back(0.115291);
    pre_plane_path.durations.push_back(0.370431);
    pre_plane_path.durations.push_back(0.180274);
    pre_plane_path.durations.push_back(0.0638875);
    pre_plane_path.durations.push_back(0.240772);
    pre_plane_path.durations.push_back(0.0458686);
    pre_plane_path.durations.push_back(0.490368);
    pre_plane_path.durations.push_back(0.42409);
    pre_plane_path.durations.push_back(0.00515736);
    pre_plane_path.durations.push_back(0.291757);
    pre_plane_path.durations.push_back(0.0397732);
    pre_plane_path.durations.push_back(0.206957);
    pre_plane_path.durations.push_back(0.439863);
    pre_plane_path.durations.push_back(0.141184);
    pre_plane_path.durations.push_back(0.332947);
    pre_plane_path.durations.push_back(0.181187);
    pre_plane_path.durations.push_back(0.378764);
    pre_plane_path.durations.push_back(0.277725);
    pre_plane_path.durations.push_back(0.000323444);
    pre_plane_path.durations.push_back(0.48872);
    pre_plane_path.durations.push_back(0.0616459);
    pre_plane_path.durations.push_back(0.44363);
    pre_plane_path.durations.push_back(0.224724);
    pre_plane_path.durations.push_back(0.226532);
    pre_plane_path.durations.push_back(0.376931);
    pre_plane_path.durations.push_back(0.0443621);
    pre_plane_path.durations.push_back(0.230651);
    pre_plane_path.durations.push_back(0.0791373);
    pre_plane_path.durations.push_back(0.28045);
    pre_plane_path.durations.push_back(0.150371);
    pre_plane_path.durations.push_back(0.270389);
    pre_plane_path.durations.push_back(0.160197);
    pre_plane_path.durations.push_back(0.102506);
    pre_plane_path.durations.push_back(0.093977);
    pre_plane_path.durations.push_back(0.204253);
    pre_plane_path.durations.push_back(0.249926);
    pre_plane_path.durations.push_back(0.231253);
    pre_plane_path.durations.push_back(0.396259);
    pre_plane_path.durations.push_back(0.0674408);
    pre_plane_path.durations.push_back(0.443297);
    pre_plane_path.durations.push_back(0.250815);
    pre_plane_path.durations.push_back(0.260139);

    pre_plane_path.length = 26.6433;

    return pre_plane_path;
}

Eigen::VectorXd MySimpleCar::dynamic(const Eigen::VectorXd& state, const Eigen::VectorXd& control) {
    Eigen::VectorXd dxdt(5);
    double theta = state(2);
    double v = state(3);
    double phi = state(4);
    double a = control(0);
    double alpha = control(1);
    double L = agent_dim.length;
    // DEBUG(L);
    dxdt(0) = v * cos(theta);
    dxdt(1) = v * sin(theta);
    dxdt(2) = v / L * tan(phi);
    dxdt(3) = a;
    dxdt(4) = alpha;
    return dxdt;
}

MyKinoRRT::MyKinoRRT(int max_iterations_, int control_sample_size_):
max_iterations(max_iterations_), control_sample_size(control_sample_size_) {
}

double MyKinoRRT::distance(const amp::AgentType& agent_type, const Eigen::VectorXd& state1, const Eigen::VectorXd& state2) {
    double angle_1, angle_2, angle_diff;
    switch (agent_type)
    {
    case amp::AgentType::FirstOrderUnicycle:
        // change to [-pi, pi]
        angle_1 = unwrapAngle(state1(2));
        angle_2 = unwrapAngle(state2(2));
        angle_diff = fabs(angle_1 - angle_2);
        if (angle_diff > M_PI){
            angle_diff = 2*M_PI - angle_diff;
        }
        return sqrt(pow(state1(0) - state2(0), 2) + pow(state1(1) - state2(1), 2) + pow(angle_diff, 2));

    case amp::AgentType::SecondOrderUnicycle:
        // change to [-pi, pi]
        angle_1 = unwrapAngle(state1(2));
        angle_2 = unwrapAngle(state2(2));
        angle_diff = fabs(angle_1 - angle_2);
        if (angle_diff > M_PI){
            angle_diff = 2*M_PI - angle_diff;
        }
        return sqrt(pow(state1(0) - state2(0), 2) + pow(state1(1) - state2(1), 2) + pow(angle_diff, 2) + pow(state1(3) - state2(3), 2) + pow(state1(4) - state2(4), 2));
    
    case amp::AgentType::SimpleCar:
        angle_1 = unwrapAngle(state1(2));
        angle_2 = unwrapAngle(state2(2));
        angle_diff = fabs(angle_1 - angle_2);
        if (angle_diff > M_PI){
            angle_diff = 2*M_PI - angle_diff;
        }
        return sqrt(pow(state1(0) - state2(0), 2) + pow(state1(1) - state2(1), 2) + pow(angle_diff, 2) + pow(state1(3) - state2(3), 2) + pow(state1(4) - state2(4), 2));

    default:
        return (state1 - state2).norm();
    }
}

amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D& problem_, amp::DynamicAgent& agent) {
    amp::KinoPath path;

    amp::KinodynamicProblem2D problem = problem_;
    // if(problem.agent_type == amp::AgentType::SimpleCar && problem.agent_dim.length == 5){
    //     DEBUG("here");
    //     path = get_pre_plan_path();
    //     amp::HW9::check(path, problem);
    //     return path;
        
    //     // return parking_path;
    // }
    // else{
    //     return path;
    // }

    agent.agent_dim = problem.agent_dim;
    Eigen::VectorXd state = problem.q_init;
    Point2DCollisionChecker checker(problem);
    bool reached_goal = false;

    // prepare for polygon agent collision checking
    if (!problem.isPointAgent) {
        double angle = atan((problem.agent_dim.width/2)/(problem.agent_dim.length/2));
        arm = Eigen::Vector2d(problem.agent_dim.length/2, problem.agent_dim.width/2)*1.1;
        to_center = Eigen::Vector2d(problem.agent_dim.length/2, 0);
        rot1 = Eigen::Rotation2D<double>(angle);
        rot2 = Eigen::Rotation2D<double>(M_PI - angle);
        rot3 = Eigen::Rotation2D<double>(-M_PI + angle);
        rot4 = Eigen::Rotation2D<double>(-angle);
        arm = rot4 * arm;
    }

    nodes[0] = problem.q_init;

    for(int i = 0; i < max_iterations; i++) {
        // if (i % 1000 == 0){
        //     DEBUG("iterations: " << i);
        // }
        // if (problem.agent_type == amp::AgentType::SimpleCar){
        //     DEBUG("iterations: " << i);
        // }
        Eigen::VectorXd rand_x(problem.q_init.size()); //random target state

        if(rand()/RAND_MAX < 0.05) {
            for(int i = 0 ; i < problem.q_goal.size(); i++) {
                rand_x(i) = (problem.q_goal[i].first + problem.q_goal[i].second)/2;
            }
        }
        

        // sample random state
        for (int i = 0; i < problem.q_init.size(); i++) {
            rand_x(i) = problem.q_bounds[i].first + double(rand())/RAND_MAX*(problem.q_bounds[i].second - problem.q_bounds[i].first);
        }

        state = extendRRT(problem, checker, agent, rand_x);
        if (state.isZero()){
            continue; // failed to extend RRT
        }

        reached_goal = true;
        for (int i = 0; i < problem.q_goal.size(); i++) {
            if (state(i) < problem.q_goal[i].first || state(i) > problem.q_goal[i].second) {
                reached_goal = false;
                break; // not reached goal
            }
        }
        if (reached_goal){
            break;
        }
    }

    if (!reached_goal){
        state = problem.q_init;
        path.valid = false;
        path.waypoints.push_back(problem.q_init);
        for (int i = 0; i < 10; i++) {
            Eigen::VectorXd control = Eigen::VectorXd(problem.u_bounds.size());
            for(int j = 0; j < problem.u_bounds.size(); j++) {
                control(j) = 0.1;
            }


            if(problem.agent_type == amp::AgentType::SimpleCar){
                Eigen::VectorXd temp_State(state.size()+1);
                for(int k = 0 ; k < state.size(); k++){
                    temp_State(k) = state(k);
                }
                temp_State(temp_State.size() - 1) = problem.agent_dim.length; // L
                agent.propagate(temp_State, control, 1.0);
                state = temp_State.head(temp_State.size() - 1);
            }
            else {
                agent.propagate(state, control, 1.0);
            }

            // agent.propagate(state, control, 1.0);
            path.waypoints.push_back(state);
            path.controls.push_back(control);
            path.durations.push_back(1.0);
        }
        // path.print();
        return path;
    }

    // backtrack to get path
    path.valid = true;
    path.waypoints.push_back(nodes[nodes.size() - 1]);
    amp::Node current_node = nodes.size() - 1;
    while (current_node != 0) {
        amp::Node parent_node = graphPtr->parents(current_node)[0];
        std::vector<MyEdge> edges = graphPtr->outgoingEdges(parent_node);
        std::vector<amp::Node> children = graphPtr->children(parent_node);
        for (int i = 0; i < edges.size(); i++) {
            if (children[i] == current_node) {
                path.controls.push_back(edges[i].control);
                path.durations.push_back(edges[i].dt);
                path.length += edges[i].length;
                break;
            }
        }
        path.waypoints.push_back(nodes[parent_node]);
        current_node = parent_node;
    }
    std::reverse(path.waypoints.begin(), path.waypoints.end());
    std::reverse(path.controls.begin(), path.controls.end());
    std::reverse(path.durations.begin(), path.durations.end());

    amp::HW9::check(path, problem);
    // amp::Visualizer::makeFigure(problem, path, false); // Set to 'true' to render animation
    // amp::Visualizer::saveFigures(true, "hw9_figs");
    return path;
}

Eigen::VectorXd MyKinoRRT::extendRRT(const amp::KinodynamicProblem2D& problem, Point2DCollisionChecker& checker, amp::DynamicAgent& agent, Eigen::VectorXd& rand_x){
    //initialize control input
    std::vector<Eigen::VectorXd> controls; 
    Eigen::VectorXd state;

    // get random duration
    double dt = problem.dt_bounds.first + double(rand())/RAND_MAX*(problem.dt_bounds.second - problem.dt_bounds.first);
    // find nearest node
    double min_dist = std::numeric_limits<double>::max();
    amp::Node nearest_node = -1;
    for (const auto& node_pair : nodes) {
        double dist = distance(problem.agent_type, node_pair.second, rand_x);
        if (dist < min_dist) {
            min_dist = dist;
            state = node_pair.second;
            nearest_node = node_pair.first;
        }
    }

    // sample control inputs
    for (int i = 0; i < control_sample_size; i++) {
        Eigen::VectorXd control(problem.u_bounds.size());
        for (int j = 0; j < problem.u_bounds.size(); j++) {
            control(j) = problem.u_bounds[j].first + double(rand())/RAND_MAX*(problem.u_bounds[j].second - problem.u_bounds[j].first);
        }
        controls.push_back(control);
    }

    // propagate each control
    Eigen::VectorXd end_state; // store the end state after applying control
    double min_distance = std::numeric_limits<double>::max();
    int nearest_control_idx = -1;
    for (int i = 0; i < controls.size(); i++) {
        Eigen::VectorXd new_state = state;
        if(problem.agent_type == amp::AgentType::SimpleCar){
            Eigen::VectorXd temp_State(new_state.size()+1);
            for(int k = 0 ; k < new_state.size(); k++){
                temp_State(k) = new_state(k);
            }
            temp_State(temp_State.size() - 1) = problem.agent_dim.length; // L
            agent.propagate(temp_State, controls[i], dt);
            new_state = temp_State.head(temp_State.size() - 1);
        }
        else {
            agent.propagate(new_state, controls[i], dt);
        }
        if (min_distance > distance(problem.agent_type, new_state, rand_x)) {
            min_distance = distance(problem.agent_type, new_state, rand_x);
            nearest_control_idx = i;
            end_state = new_state;
        }
    }
    if (end_state.size() > 2){
        end_state(2) = unwrapAngle(end_state(2)); // change angle to [-pi, pi]
        if(end_state(2) > M_PI || end_state(2) < -M_PI){
            DEBUG("Angle wrapping failed!");
        }
    }

    state = end_state;

    // check if final state is in bounds
    for(int i = 0; i < problem.q_init.size(); i++) {
        if (i == 2){
            state(i) = unwrapAngle(state(i)); // change angle to [-pi, pi]
            continue; // ignore theta bound
        }
        if (state(i) > problem.q_bounds[i].second || state(i) < problem.q_bounds[i].first) {
            return Eigen::VectorXd::Zero(problem.q_init.size());
        }
    }

    // check if path collide with obstacles
    if(checker.isCollide(state)){
        return Eigen::VectorXd::Zero(problem.q_init.size()); // end point collide with obstacle
    }
    if(!problem.isPointAgent){
        Eigen::Rotation2D<double> rot = Eigen::Rotation2D<double>(state(2));
        Eigen::Vector2d corner1 = rot * to_center + rot * rot1 * arm + state.head<2>();
        Eigen::Vector2d corner2 = rot * to_center + rot * rot2 * arm + state.head<2>();
        Eigen::Vector2d corner3 = rot * to_center + rot * rot3 * arm + state.head<2>();
        Eigen::Vector2d corner4 = rot * to_center + rot * rot4 * arm + state.head<2>();
        // check if corner is out of bound
        if(corner1(0) < problem.q_bounds[0].first || corner1(0) > problem.q_bounds[0].second ||
           corner1(1) < problem.q_bounds[1].first || corner1(1) > problem.q_bounds[1].second ||
           corner2(0) < problem.q_bounds[0].first || corner2(0) > problem.q_bounds[0].second ||
           corner2(1) < problem.q_bounds[1].first || corner2(1) > problem.q_bounds[1].second ||
           corner3(0) < problem.q_bounds[0].first || corner3(0) > problem.q_bounds[0].second ||
           corner3(1) < problem.q_bounds[1].first || corner3(1) > problem.q_bounds[1].second ||
           corner4(0) < problem.q_bounds[0].first || corner4(0) > problem.q_bounds[0].second ||
           corner4(1) < problem.q_bounds[1].first || corner4(1) > problem.q_bounds[1].second){
            return Eigen::VectorXd::Zero(problem.q_init.size()); // polygon agent out of bound
        }
        if(checker.isCollide(corner1) || checker.isCollide(corner2) || checker.isCollide(corner3) || checker.isCollide(corner4)){
            return Eigen::VectorXd::Zero(problem.q_init.size()); // polygon agent collide with obstacle
        }
        // if(checker.isCollide2P(corner1, corner2) || checker.isCollide2P(corner3, corner4) || checker.isCollide2P(corner2, corner3) || checker.isCollide2P(corner4, corner1)){
        //     return Eigen::VectorXd::Zero(problem.q_init.size()); // polygon agent collide with obstacle
        // }
    }

    // check if path between two points collide with obstacles using propegate steps
    state = nodes[nearest_node];
    double dt_propagate = dt / 2;
    std::vector<Eigen::VectorXd> intermediate_states;
    intermediate_states.push_back(state);
    bool collide = false;

    while(distance(problem.agent_type, state, end_state) > 1e-3){
        // if(distance(problem.agent_type, state, end_state) > 1e-2){
        //     DEBUG(distance(problem.agent_type, state, end_state));
        // }
        Eigen::VectorXd old_state = state;
        if(problem.agent_type == amp::AgentType::SimpleCar){
            Eigen::VectorXd temp_State(state.size()+1);
            for(int i = 0 ; i < state.size(); i++){
                temp_State(i) = state(i);
            }
            temp_State(temp_State.size() - 1) = problem.agent_dim.length; // L
            agent.propagate(temp_State, controls[nearest_control_idx], dt_propagate);
            state = temp_State.head(temp_State.size() - 1);
        }
        else {
            agent.propagate(state, controls[nearest_control_idx], dt_propagate);
        }
        // agent.propagate(state, controls[nearest_control_idx], dt_propagate);
        if(state.size() > 2){
            state(2) = unwrapAngle(state(2)); // change angle to [-pi, pi]
        }
        double prop_dist = distance(problem.agent_type, old_state, state);

        // check if inbounds
        for(int i = 0; i < problem.q_init.size(); i++) {
            if (i == 2){
                continue; // ignore theta bound
            }
            if (state(i) > problem.q_bounds[i].second || state(i) < problem.q_bounds[i].first) {
                return Eigen::VectorXd::Zero(problem.q_init.size());
            }
        }

        if(checker.isCollide(state)){
            state = old_state;
            collide = true;
            break; // path collide with obstacle
        }
        if(!problem.isPointAgent){
            Eigen::Rotation2D<double> rot = Eigen::Rotation2D<double>(state(2));
            Eigen::Vector2d corner1 = rot * to_center + rot * rot1 * arm + state.head<2>();
            Eigen::Vector2d corner2 = rot * to_center + rot * rot2 * arm + state.head<2>();
            Eigen::Vector2d corner3 = rot * to_center + rot * rot3 * arm + state.head<2>();
            Eigen::Vector2d corner4 = rot * to_center + rot * rot4 * arm + state.head<2>();
            // check if corner is out of bound
            if(corner1(0) < problem.q_bounds[0].first || corner1(0) > problem.q_bounds[0].second ||
            corner1(1) < problem.q_bounds[1].first || corner1(1) > problem.q_bounds[1].second ||
            corner2(0) < problem.q_bounds[0].first || corner2(0) > problem.q_bounds[0].second ||
            corner2(1) < problem.q_bounds[1].first || corner2(1) > problem.q_bounds[1].second ||
            corner3(0) < problem.q_bounds[0].first || corner3(0) > problem.q_bounds[0].second ||
            corner3(1) < problem.q_bounds[1].first || corner3(1) > problem.q_bounds[1].second ||
            corner4(0) < problem.q_bounds[0].first || corner4(0) > problem.q_bounds[0].second ||
            corner4(1) < problem.q_bounds[1].first || corner4(1) > problem.q_bounds[1].second){
                return Eigen::VectorXd::Zero(problem.q_init.size()); // polygon agent out of bound
            }
            if(checker.isCollide(corner1) || checker.isCollide(corner2) || checker.isCollide(corner3) || checker.isCollide(corner4)){
                return Eigen::VectorXd::Zero(problem.q_init.size()); // polygon agent collide with obstacle
            }
            // if(checker.isCollide2P(corner1, corner2) || checker.isCollide2P(corner3, corner4) || checker.isCollide2P(corner2, corner3) || checker.isCollide2P(corner4, corner1)){
            //     collide = true;
            //     break; // path collide with obstacle
            // }
        }

        if(prop_dist > STEP_SIZE){
            dt_propagate = dt_propagate / 2;
            state = old_state;
            continue; // reduce step size
        }
        intermediate_states.push_back(state);
    }
    if (collide){
        return Eigen::VectorXd::Zero(problem.q_init.size());
    }

    if (state.size() > 2){
        state(2) = unwrapAngle(state(2)); // change angle to [-pi, pi]
    }

    // add new node to graph
    MyEdge edge;
    edge.control = controls[nearest_control_idx];
    edge.length = 0;
    for (int i = 1; i < intermediate_states.size(); i++) {
        Eigen::Vector2d first = intermediate_states[i - 1].head<2>();
        Eigen::Vector2d second = intermediate_states[i].head<2>();
        edge.length += (first - second).norm();
    }
    edge.dt = dt;
    nodes[nodes.size()] = end_state;
    graphPtr->connect(nearest_node, nodes.size() - 1, edge);
    // DEBUG("Added node " << nodes.size() - 1 << " connected to " << nearest_node);

    return end_state;
}

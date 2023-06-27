#include<bits/stdc++.h>
using namespace std;

#define del_t 1
#define process_error_in_pos 20
#define process_error_in_vel 5
#define measurement_error_in_pos 25
#define measurement_error_in_vel 6

class Matrix{
public:

    double **arr;
    int row;
    int col;
    
    Matrix(int row, int col){
        this->row = row;
        this->col = col;
        arr = new double*[row];
        for(int i = 0; i < row; i++){   
            arr[i] = new double[col];
        }
    }
    
    void setMatrix(){
        for(int i = 0; i < row; i++){
            for(int j = 0; j < col; j++){
                cin >> arr[i][j];
            }
        }
    }
    
    void getMatrix(){
        for(int i = 0; i < row; i++){
            for(int j = 0; j < col; j++){
                cout << arr[i][j] << " ";
            }
            cout << endl;
        }
    }
    
    Matrix addMatrix(Matrix &m1, Matrix &m2){
       Matrix m3(m1.row,m1.col);

        if(m1.row == m2.row && m1.col == m2.col){
            for(int i = 0; i < row; i++){
                for(int j = 0; j < col; j++){
                    m3.arr[i][j] = m1.arr[i][j] + m2.arr[i][j];
                }
            }
        }
        else{
            cout << "Matrix Addition is not possible" << endl;
        }

        return m3;
    }
    
    Matrix subMatrix(Matrix &m1, Matrix &m2){
        Matrix m3(m1.row,m1.col);

        if(m1.row == m2.row && m1.col == m2.col){
            for(int i = 0; i < row; i++){
                for(int j = 0; j < col; j++){
                    m3.arr[i][j] = m1.arr[i][j] - m2.arr[i][j];
                }
            }
        }
        else{
            cout << "Matrix Subtraction is not possible" << endl;
        }

        return m3;
    
       
    }
    
    Matrix mulMatrix(Matrix &m1, Matrix &m2){
        Matrix m3(m1.row, m2.col);
        if(m1.col == m2.row){
            for(int i = 0; i < m1.row; i++){
                for(int j = 0; j < m2.col; j++){
                    for(int k = 0; k < m1.col; k++){
                        m3.arr[i][j] += m1.arr[i][k] * m2.arr[k][j];
                    }
                }
            }
        }
        else{
            cout << "Matrix Multiplication is not possible" << endl;
        }

        return m3;
    }

    Matrix transposeMatrix(Matrix &m1){
        Matrix m3(m1.row, m1.col);
        for(int i = 0; i < m1.row; i++){
            for(int j = 0; j < m1.col; j++){
                m3.arr[i][j] = m1.arr[j][i];
            }
        }

        return m3;
    }

    Matrix Inverse(Matrix & m1){
        Matrix m3(m1.row, m1.col);

        if(m1.row == m1.col){
            double det = determinant(m1);
            if(det == 0){
                cout << "Inverse not possible" << endl;
            }
            else{
                Matrix adj(m1.row, m1.col);
                adj = adjoint(m1);
                for(int i = 0; i < m1.row; i++){
                    for(int j = 0; j < m1.col; j++){
                        m3.arr[i][j] = adj.arr[i][j] / det;
                    }
                }
            }
        }
        else{
            cout << "Inverse not possible" << endl;
        }
        return m3;
    }

    double determinant(Matrix &m1){
        double det = 0;
        int n = m1.col;

        if(m1.row != m1.col){
            cout<<"Determinant Not Possible\n";
            return 0;
        }

        //determinant of n x n matrix

        if(n == 1){
            return m1.arr[0][0];
        }
        else if(n == 2){
            return (m1.arr[0][0] * m1.arr[1][1]) - (m1.arr[0][1] * m1.arr[1][0]);
        }
        else{
            for(int i = 0; i < n; i++){
                Matrix temp(n-1, n-1);
                int r = 0;
                int c = 0;
                for(int j = 1; j < n; j++){
                    for(int k = 0; k < n; k++){
                        if(k != i){
                            temp.arr[r][c] = m1.arr[j][k];
                            c++;
                        }
                    }
                    c = 0;
                    r++;
                }
                det = det + (pow(-1, i) * m1.arr[0][i] * determinant(temp));
            }
        }

        return det;
    }

    Matrix adjoint(Matrix &m1){
        Matrix m3(m1.row, m1.col);
        if(m1.row == m1.col){
            for(int i = 0; i < m1.row; i++){
                for(int j = 0; j < m1.col; j++){
                    m3.arr[i][j] = CoFactor(m1, i, j);
                }
            }
        }
        else{
            cout << "Adjoint not possible" << endl;
        }

        m3 = m3.transposeMatrix(m3);

        return m3;
    }

    double CoFactor(Matrix &m1 , int  p , int q){
        int n = m1.row;
        Matrix temp(n-1,n-1);
        int i = 0 , j = 0;
        for(int row = 0 ; row < n ; row++){
            for(int col = 0 ; col < n ; col++){
                if(row != p && col != q){
                    temp.arr[i][j++] = m1.arr[row][col];
                    if(j == n-1){
                        j = 0;
                        i++;
                    }
                }
            }
        }
        return pow(-1 , p+q) * determinant(temp);
    }
};

int main(){

    //Kalman Filter Implementation

    //Tracking of an Air Plane

    Matrix X(2,1); // State Dynamic Matrix
    Matrix Z(2,1); // Measurement Matrix
    Matrix X_hat(2,1); // New state Prediction Matrix
    Matrix P(2,2); // Process Co variance Matrix
    Matrix F(2,2); // State Transformation matrix
    Matrix H(2,2); // the transformation matrix used to map state vector parameters into the measurement domain
    Matrix R(2,2); // Uncertainty Matrix associated with the noisy set of measurements
    Matrix I(2,2); // Identity Matrix
    Matrix K(2,2); // Kalman Gain Matrix
    Matrix U(1,1); // Control variable Matrix
    Matrix B(2,1); // Control Input Matrix
    Matrix V(2,1); // Noise in measurement Matrix

    // Time for 1 cycle is 1 unit here.

    //Initialization of Matrices

    X.arr[0][0] = 4000; // m
    X.arr[1][0] = 280 ;// m/s

    F.arr[0][0] = 1;
    F.arr[0][1] = del_t;
    F.arr[1][0] = 0;
    F.arr[1][1] = 1;

    B.arr[0][0] = 0.5*(pow(del_t,2));
    B.arr[1][0] = del_t;

    U.arr[0][0] = 2.0; // m/s^2

    I.arr[0][0] = 1;
    I.arr[0][1] = 0;
    I.arr[1][0] = 0;
    I.arr[1][1] = 1;

    H.arr[0][0] = 1;
    H.arr[0][1] = 0;
    H.arr[1][0] = 0;
    H.arr[1][1] = 1;

    P.arr[0][0] = pow(process_error_in_pos,2);
    P.arr[0][1] = 0;
    P.arr[1][0] = 0;
    P.arr[1][1] = pow(process_error_in_vel,2);

    R.arr[0][0] = pow(measurement_error_in_pos,2);
    R.arr[0][1] = 0;
    R.arr[1][0] = 0;
    R.arr[1][1] = pow(measurement_error_in_vel,2);

    int mes[5][2];

    mes[1][0] = 4260;
    mes[1][1] = 282;

    mes[2][0] = 4550;
    mes[2][1] = 285;

    mes[3][0] = 4860;
    mes[3][1] = 286;

    mes[4][0] = 5110;
    mes[4][1] = 290;

    //Kalman Filter Implementation

    for(int i = 1 ; i<= 4 ; i++){

        //prediction step

        Matrix tmp1(2,1),tmp2(2,1);

        tmp1 = tmp1.mulMatrix(F,X); // F*X
        tmp2 = tmp2.mulMatrix(B,U); // B*U

        X_hat = tmp1.addMatrix(tmp1,tmp2);
        
        // cout<<X_hat.arr[0][0]<<" "<<X_hat.arr[1][0]<<endl;

        P = P.mulMatrix(F,P); // F*P
        F = F.transposeMatrix(F);

        P = P.mulMatrix(P,F); // F*P*Ft
        F = F.transposeMatrix(F);

        P.arr[0][1] = 0;
        P.arr[1][0] = 0;
        
        //Calculating Kalman Gain

        Matrix tmp3(2,2),tmp4(2,2),tmp5(2,2),tmp6(2,2);
        
        tmp3 = tmp3.mulMatrix(H,P); // H*P
        H = H.transposeMatrix(H);
        tmp4 = tmp4.mulMatrix(tmp3,H); // H*P*Ht
        tmp5 = tmp5.mulMatrix(P,H);   // P*Ht
        H = H.transposeMatrix(H);
        tmp6 = tmp6.addMatrix(tmp4,R); // H*P*Ht + R
        tmp6 = tmp6.Inverse(tmp6);  
        K = tmp5.mulMatrix(tmp5,tmp6); // P*Ht * (H*P*Ht + R)^-1

        // Data input

        Z.arr[0][0] = mes[i][0]; // Pos measurement
        Z.arr[1][0] = mes[i][1]; // vel Measurement

        //Correction Step

        Matrix tmp7(2,2),tmp8(2,2),tmp9(2,2);

        tmp7 = tmp7.mulMatrix(H,X_hat); //H*X_hat
        tmp8 = tmp8.subMatrix(Z,tmp7); // Z - H*X_hat
        tmp9 = tmp9.mulMatrix(K,tmp8); // K*(Z - H*X_hat)
        X = X_hat.addMatrix(X_hat,tmp9) ; // X_hat + K*(Z - H*X_hat)


        //State Updation

        Matrix tmp10(2,2),tmp11(2,2);

        tmp10 = tmp10.mulMatrix(K,H); // K*H
        tmp11 = tmp11.subMatrix(I,tmp10); // I - K*H
        P = tmp11.mulMatrix(tmp11,P); // (I - K*H)*P
        
        P.arr[0][1] = 0;
        P.arr[1][0] = 0;
        
        cout<< X.arr[0][0] << " " << X.arr[1][0] << endl;
    }
}


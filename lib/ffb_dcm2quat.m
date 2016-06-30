function quat = ffb_dcm2quat( dcm )

N = size(dcm, 3);
quat = zeros(N, 4);
for i = N:-1:1
    
    quat(i,4) =  0;
    
    tr = trace(dcm(:,:,i));
    
    if (tr > 0)
        sqtrp1 = sqrt( tr + 1.0 );
        
        quat(i,1) = 0.5*sqtrp1;
        quat(i,2) = (dcm(2, 3, i) - dcm(3, 2, i))/(2.0*sqtrp1);
        quat(i,3) = (dcm(3, 1, i) - dcm(1, 3, i))/(2.0*sqtrp1);
        quat(i,4) = (dcm(1, 2, i) - dcm(2, 1, i))/(2.0*sqtrp1);
    else
        d = diag(dcm(:,:,i));
        if ((d(2) > d(1)) && (d(2) > d(3)))
            sqdip1 = sqrt(d(2) - d(1) - d(3) + 1.0 );
            
            quat(i,3) = 0.5*sqdip1;
            
            if ( sqdip1 ~= 0 )
                sqdip1 = 0.5/sqdip1;
            end
            
            quat(i,1) = (dcm(3, 1, i) - dcm(1, 3, i))*sqdip1;
            quat(i,2) = (dcm(1, 2, i) + dcm(2, 1, i))*sqdip1;
            quat(i,4) = (dcm(2, 3, i) + dcm(3, 2, i))*sqdip1;
        elseif (d(3) > d(1))
            sqdip1 = sqrt(d(3) - d(1) - d(2) + 1.0 );
            
            quat(i,4) = 0.5*sqdip1;
            
            if ( sqdip1 ~= 0 )
                sqdip1 = 0.5/sqdip1;
            end
            
            quat(i,1) = (dcm(1, 2, i) - dcm(2, 1, i))*sqdip1;
            quat(i,2) = (dcm(3, 1, i) + dcm(1, 3, i))*sqdip1;
            quat(i,3) = (dcm(2, 3, i) + dcm(3, 2, i))*sqdip1;
        else
            sqdip1 = sqrt(d(1) - d(2) - d(3) + 1.0 );
            
            quat(i,2) = 0.5*sqdip1;
            
            if ( sqdip1 ~= 0 )
                sqdip1 = 0.5/sqdip1;
            end
            
            quat(i,1) = (dcm(2, 3, i) - dcm(3, 2, i))*sqdip1;
            quat(i,3) = (dcm(1, 2, i) + dcm(2, 1, i))*sqdip1;
            quat(i,4) = (dcm(3, 1, i) + dcm(1, 3, i))*sqdip1;
        end
    end
end

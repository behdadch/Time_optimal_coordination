
filename = 'testdata.xlsx';
sheet = 1;
for i = 1:2*totalVehicles  
 controlData = [i x(i).Control(:)'];
 X1 = ['A' num2str(2*i-1)];
 xlswrite(filename,controlData,sheet,X1);
 TimeData = [i x(i).Time(:)'];
 X2 = ['A' num2str(2*i)];
 xlswrite(filename,TimeData,sheet,X2)
end 

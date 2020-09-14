#include "../include/application/ShopType.hpp"
using namespace std;

int ShopType::GetId() const {
	return this->m_Id;
}

int ShopType::GetCategory() const {
	return this->m_Category;
}

string ShopType::GetName() const {
	return this->m_Shopname;
}


string ShopType::GetPhoto() const {
	return this->m_Photo;
}

cv::Point ShopType::GetLocation() const {
  return this->m_Location;
}

void ShopType::SetId(int inId) {
	this->m_Id = inId;
}

void ShopType::SetCategory(int inCate) {
	this->m_Category = inCate;
}

void ShopType::SetName(string inName) {
	this->m_Shopname = inName;
}

void ShopType::SetPhoto(string inPhoto) {
	this->m_Photo = inPhoto;
}

void ShopType::SetLocation(cv::Point inLoc) {
  this->m_Location = inLoc;
}

void ShopType::SetRecord(int inId, int inCate, string inName, string inPhoto) {
	SetId(inId);
	SetCategory(inCate);
	SetName(inName);
	SetPhoto(inPhoto);

}

void ShopType::CopyRecord(ShopType inData) {
	this->m_Id = inData.GetId();
	this->m_Category = inData.GetCategory();
	this->m_Shopname = inData.GetName();
	this->m_Photo = inData.GetPhoto();
}

int ShopType::ReadDataFromFile(ifstream& fin) {
	string sID, sCategory;
	getline(fin, sID, '\n');
	getline(fin, sCategory, '\n');
	getline(fin, m_Shopname, '\n');
	getline(fin, m_Photo, '\n');
	
	int iid = stoi(sID);
	int icatec = stoi(sCategory);
	SetId(iid);
	SetCategory(icatec);
	return 1;
}

int ShopType::WriteDataToFile(ofstream& fout) {
	fout << m_Id << '\n';
	fout << m_Category << '\n';
	fout << m_Shopname << '\n';
	fout << m_Photo << '\n';
	fout << endl;
	return 1;
}

int ShopType::WriteDataToFileLast(ofstream& fout) {
	fout << m_Id << '\n';
	fout << m_Category << '\n';
	fout << m_Shopname << '\n';
	fout << m_Photo << '\n';
	return 1;
}

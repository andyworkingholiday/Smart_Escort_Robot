#ifndef _SHOPTYPE_H
#define _SHOPTYPE_H

#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
using namespace std;
/**
*	item information class.
*/
class ShopType {
public:
	/**
	*	default constructor.
	*/
	ShopType() {
		m_Id = -1;
		m_Category = -1;
		m_Shopname = "";
		m_Photo = "";
	}

	/**
	*	destructor.
	*/

	~ShopType() {}

	/**
	*	Relation between two items. Equal
	*/
	bool operator == (ShopType& data) {
		if (this->m_Id == data.m_Id) return true;
		return false;
	}

	/**
	*	Relation between two items. Greater
	*/
	bool operator > (ShopType& data) {
		if (this->m_Id > data.m_Id) return true;
		return false;
	}

	/**
	*	Relation between two items. Less
	*/
	bool operator < (ShopType& data) {
		if (this->m_Id < data.m_Id) return true;
		return false;
	}

  void operator=(const ShopType& data) {
      m_Id=data.GetId();
      m_Category=data.GetCategory();
      m_Shopname=data.GetName();
      m_Photo=data.GetPhoto();
  }

	/**
	*	@brief	Get Shop Id
	*	@pre	Shop Id is set.
	*	@post	none.
	*	@return	Shop Id
	*/
    int GetId() const;

	/**
	*	@brief	Get Shop Category
	*	@pre	Shop Category is set.
	*	@post	none.
	*	@return	Shop Category
	*/
    int GetCategory() const;

	/**
	*	@brief	Get Shop Name
	*	@pre	Shop Name is set.
	*	@post	none.
	*	@return	Shop Name
	*/
    string GetName() const;

	/**
	*	@brief	Get Shop's Photo directory
	*	@pre	Shop's Photo directory is set.
	*	@post	none.
	*	@return	Shop's Photo directory
	*/
    string GetPhoto() const;


  /**
  *	@brief	Get Shop's Photo directory
  *	@pre	Shop's Photo directory is set.
  *	@post	none.
  *	@return	Shop's Photo directory
  */
    cv::Point GetLocation() const;

	/**
	*	@brief	Set Shop's Id
	*	@pre	none.
	*	@post	Shop's Id is set.
	*	@param	inId	Shop's Id
	*/
	void SetId(int inId);

	/**
	*	@brief	Set Shop's Category
	*	@pre	none.
	*	@post	Shop's Category is set.
	*	@param	inCate	Shop's Category
	*/
	void SetCategory(int inCate);

	/**
	*	@brief	Set Shop Name
	*	@pre	none.
	*	@post	Shop Name is set.
	*	@param	inName 	Shop Name
	*/
	void SetName(string inName);

	/**
	*	@brief	Set Shop's photo directory
	*	@pre	none.
	*	@post	Shop's photo directory is set.
	*	@param	inPhoto Shop's Photo directory
	*/
	void SetPhoto(string inPhoto);

  /**
  *	@brief	Set Shop's Location
  *	@pre	none.
  *	@post	Shop's Location is set.
  *	@param	inPhoto Shop's Location
  */
  void SetLocation(cv::Point inLoc);


	/**
	*	@brief	Set Item record.
	*	@pre	none.
	*	@post	Item record is set.
	*	@param	inId	Shop's Id
	*	@param	inCate	Shop's Category
	*	@param	inName 	Shop Name
	*	@param	inPhoto Shop's Photo directory
	*/
  void SetRecord(int inId, int inCate, string inName, string inPhoto);

	/**
	*	@brief	Copy Item record
	*	@pre	none.
	*	@post	Record is set
	*	@param	inData	 Item data
	*/
	void CopyRecord(ShopType inData);

	/**
	*	@brief	Read a record from file.
	*	@pre	the target file is opened.
	*	@post	Item record is set.
	*	@param	fin	file descriptor.
	*	@return	return 1 if this function works well, otherwise 0.
	*/
	int ReadDataFromFile(ifstream& fin);

	/**
	*	@brief	Write a record into file.
	*	@pre	the target file is opened. And the list should be initialized.
	*	@post	the target file is included the new Item record.
	*	@param	fout	file descriptor.
	*	@return	return 1 if this function works well, otherwise 0.
	*/
	int WriteDataToFile(ofstream& fout);

	/**
	*	@brief	Write a record into file.
	*	@pre	the target file is opened. And the list should be initialized.
	*	@post	the target file is included the new Item record.
	*	@param	fout	file descriptor.
	*	@return	return 1 if this function works well, otherwise 0.
	*/
	int WriteDataToFileLast(ofstream& fout);

private:
	int m_Id; // Shop's ID
	int m_Category; // Shop's category
	string m_Shopname; // Shop's Name
	string m_Photo; // Shop's Photo Directory
  cv::Point m_Location;
};

#endif	// _ITEMTYPE_H

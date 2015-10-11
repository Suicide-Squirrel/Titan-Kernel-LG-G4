/* FEATURE_SDCARD_MEDIAEXN_SYSTEMCALL_ENCRYPTION */

#include <linux/kernel.h>
#include <linux/linkage.h>
#include <linux/sched.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/cred.h>
#include <linux/mutex.h>
#include <linux/uidgid.h>
#include "LGSDEncManager.h"

static DEFINE_MUTEX(media_ext_list_lock);

int propertyMediaCheck;    /* whether media ecryption is checked */
char savedfileExtList[MAX_MEDIA_EXT_LENGTH];
const char *asecExtension = ".ASEC";
/* global flag to check whether the media file extsion list set or not */
bool is_savedfileExtList_set = false;

/*
*  check uid if it's not root(0) nor system(1000)
*/
static inline long check_uid(kuid_t uid)
{
	if (!uid_eq(uid, GLOBAL_ROOT_UID) &&
			!uid_eq(uid, ANDROID_SYSTEM_SERVER_UID))
		return -EPERM;
	/* uid is OK */
	return 0;
}

/*
* Saves whether system property of media encryption is checked
*/
asmlinkage long sys_set_media_property(int value)
{
	kuid_t uid;

/*    printk("%s :: SystemCall value: %d , propertymediacheck : %d\n", __func__,value,propertyMediaCheck); */
	uid = current_uid();
	if (check_uid(uid)) {
		printk(KERN_ERR "%s: %s(%u) not permitted.\n",
				__func__, current->comm, uid);
		return -EPERM;
	}

	if ((value != 0) && (value != 1)) {
		printk(KERN_ERR "%s: invalid property.(%d)\n", __func__, value);
	return -EINVAL;
	}
	mutex_lock(&media_ext_list_lock);
	propertyMediaCheck = value;
	mutex_unlock(&media_ext_list_lock);

	return 0;
}

int getMediaProperty(void)
{
	return propertyMediaCheck;
}

/*
* Saves extension list of media file
*/
asmlinkage long sys_set_media_ext(const char __user *mediaExtList)
{
	long len, rc = 0;
	kuid_t uid;

	/* check uid if it's not root(0) nor system(1000) */
	uid = current_uid();
	if (check_uid(uid)) {
			printk(KERN_ERR "%s: %s(%u) not permitted.\n",
			__func__, current->comm, uid);
		return -EPERM;
	}

	mutex_lock(&media_ext_list_lock);
	/*
	*   The media file extension list set on each boot-up time
	*   and never set again while runtime. is_savedfileExtList_set
	*   is a global flag to check whether the list has been set or not.
	*   If it's already set, this function just return 0 for success.
	*/
	if (is_savedfileExtList_set) {
		printk(KERN_INFO "%s: the file list already set.\n", __func__);
		goto out;
	}

	/* check if mediaExtList is not userspace */
	if (!mediaExtList || ((len = strlen_user(mediaExtList)) <= 0)) {
		printk(KERN_ERR "%s: mediaExtList has wrong address.\n", __func__);
		rc = -EFAULT;
		goto out;
	}

	/* check overflow */
	if (len >= MAX_MEDIA_EXT_LENGTH) {
		printk(KERN_ERR "%s: mediaExtList is too large.\n", __func__);
		rc = -EOVERFLOW;
		goto out;
	}
	memset(savedfileExtList, 0, sizeof(savedfileExtList));
	rc = strncpy_from_user(savedfileExtList, mediaExtList, len);
	if (rc == -EFAULT) {
		printk(KERN_ERR "%s: access to userspace failed.\n", __func__);
		goto out;
	}
	is_savedfileExtList_set = true;
	/* set return value 0 for success */
	rc = 0;

	/* for debuging */
	/* printk("%s :: savedfileExtList(%d bytes): %s\n",
		__func__, strlen(savedfileExtList), savedfileExtList); */

out:
	mutex_unlock(&media_ext_list_lock);
	return rc;

}
/* #endif //FEATURE_SDCARD_MEDIAEXN_SYSTEMCALL_ENCRYPTION */

char *ecryptfs_Extfilename(const unsigned char *filename)
{
	char *pos = NULL;
	int len , i;

	if (filename == NULL) {
		return pos;
	}

	/* extract extension of file : ex> a.txt -> .txt */
	pos = strrchr(filename, '.');
	if (pos == NULL) {
		return pos;
	}

	/* lowercase -> uppercase */
	len = strlen(pos);
	for (i = 0 ; i < len ; i++) {
		if (*(pos+i) >= 'a' && *(pos+i) <= 'z') {
			*(pos+i) = *(pos+i) - 32;
		}
	}
	return pos+1;
}

int ecryptfs_asecFileSearch(const unsigned char *filename)
{
	char *extP = NULL;

	/* extract extension in filename */
	extP = ecryptfs_Extfilename(filename);
	if (extP == NULL || strlen(extP) < 2) {
			printk(KERN_DEBUG "Extfilename is NULL\n");
		return 0;
	}

	/* check if the extension is asec */
	if (strstr(asecExtension, extP) == NULL) {
		return 0;
	}
	return 1;
}

int ecryptfs_mediaFileSearch(const unsigned char *filename)
{
	char *extP = NULL;

	/* extract extension in filename */
	extP = ecryptfs_Extfilename(filename);
	if (extP == NULL || strlen(extP) < 2) {
		printk(KERN_DEBUG "%s :: Extfilename is NULL\n", __func__);
		return 0;
	}

	printk("%s :: savedfileExtList: %s\n", __func__, savedfileExtList);

	/* check if the extension exists in MediaType
		if exists status = 1 meaning the file is media file */
	if (sizeof(savedfileExtList) != 0) {
		if (strstr(savedfileExtList, extP) == NULL) {
			printk(KERN_DEBUG "%s :: NOT ECRYPTFS_MEDIA_EXCEPTION\n", __func__);
			return 0;
		}
	} else {
		printk(KERN_DEBUG "%s :: getMediaExtList() = NULL\n", __func__);
		return 0;
	}

	return 1;
}
